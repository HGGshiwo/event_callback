import asyncio
import copy
import json
import logging
import threading
import uuid
from contextlib import asynccontextmanager
from pathlib import Path

try:
    from typing import TypeAlias
except ImportError:
    from typing_extensions import TypeAlias

from typing import Any, Callable, Dict, Type, Union

import uvicorn
from fastapi import FastAPI, Request, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.middleware.gzip import GZipMiddleware
from uvicorn.config import Config

from event_callback.components.http.ui_config import BaseUIConfig

from .utils import dict2route, request2dict

try:
    import rospy
    from event_callback_msg.srv import StringSrv, StringSrvResponse
    from std_msgs.msg import String
except:
    String: TypeAlias = Any
    Empty: TypeAlias = Any
    StringSrv: TypeAlias = Any
    StringSrvRequest: TypeAlias = Any
    StringSrvResponse: TypeAlias = Any


import logging
from typing import List

# 组件基类导入
from event_callback.core import BaseComponent, BaseEvent

logger = logging.getLogger(__name__)


class FilterLogMiddleware(BaseHTTPMiddleware):
    """FastAPI日志过滤中间件：屏蔽指定路径的访问日志，减少冗余输出"""

    def __init__(
        self, app: Any, dispatch: Any = None, paths_to_exclude: List[str] = None
    ):
        super().__init__(app, dispatch)
        self._paths_to_exclude = paths_to_exclude

    async def dispatch(self, request: Request, call_next):
        # 需过滤的路径前缀，保留原有配置
        logger = logging.getLogger("uvicorn.access")
        logger.disabled = any(p in request.url.path for p in self._paths_to_exclude)
        response = await call_next(request)
        logger.disabled = False
        return response


class RequestHandler:
    """ROS服务请求处理器：封装FastAPI请求解析与ROS Service调用，实现动态路由映射"""

    def __init__(self, path: str, method: str, topic: str):
        self.path = path
        self.method = method
        self.topic = topic
        rospy.wait_for_service(topic, timeout=10)  # 增加超时，避免永久阻塞
        self.ros_srv_proxy = rospy.ServiceProxy(topic, StringSrv)

    async def __call__(self, request: Request):
        """解析FastAPI请求（路径/查询/体参数），调用ROS服务并返回JSON响应"""
        # 异常捕获，放置调用服务失败导致500
        try:
            request_json = await request2dict(request)
            loop = asyncio.get_event_loop()
            ros_resp = await loop.run_in_executor(
                None, lambda: self.ros_srv_proxy(request=json.dumps(request_json))
            )
            return json.loads(ros_resp.response)
        except Exception as e:
            import traceback

            return dict(detail=traceback.format_exc(), status="error", msg=str(e))


class POSTEvent(BaseEvent):
    def __call__(self, url: str):
        return super().__call__(url=url, method="POST")


class GETEvent(BaseEvent):
    def __call__(self, url: str):
        return super().__call__(url=url, method="GET")


class MessageEvent(BaseEvent):
    def __call__(self, route: str = None):
        kwargs = {}
        if route is not None:
            kwargs["route"] = route
        return super().__call__(**kwargs)


class HTTPComponent(BaseComponent):
    """FastAPI桥接组件：基于FastAPI实现HTTP/WS服务，支持与ROS的动态路由映射"""

    on_post = POSTEvent()
    on_get = GETEvent()
    on_message = MessageEvent()
    on_connect = BaseEvent()
    on_disconnect = BaseEvent()

    def __init__(
        self,
        register: bool = False,
        static_dir: Union[str, Path, None] = None,
        home_dir: Union[str, Path, None] = None,
        websocket_topic: str = "ws",
        log_exclude_path: List = None,
        router: Callable = None,
    ):
        """
        初始化FastAPI-ROS组件

        :param config: 组件配置参数，支持：

            - register: 布尔值，是否启用ROS register服务和do_register触发（默认False）
            - static_dir: 静态文件目录（默认上级目录的static）
            - log_exclude_path: 忽略日志记录的路径
            - websocket_topic: 监听 websocket topic名称
        """
        super().__init__()

        if log_exclude_path is None:
            log_exclude_path = []

        # 组件核心配置解析
        self.enable_register = register
        self.router = router

        default_static_dir = Path(__file__).parent.parent / "static"
        static_dir = default_static_dir if static_dir is None else static_dir
        self.static_dir = Path(static_dir)
        default_home_dir = Path(__file__).parents[4] / "event-callback-app" / "dist"
        home_dir = home_dir if home_dir is not None else default_home_dir
        self.home_dir = Path(home_dir)

        self.websockt_topic = websocket_topic
        self.log_exclude_path = log_exclude_path

        self.server_ready = threading.Event()

        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self.server_ready.set()  # 点亮绿灯！
            yield
            logger.info("FastAPI Server shutdown...")

        # FastAPI核心实例变量
        self.app = FastAPI(lifespan=lifespan)  # FastAPI应用实例

        self.loop = None  # FastAPI异步事件循环
        self.server_thread = None  # FastAPI服务线程（与ROS解耦）

        # 初始化FastAPI（中间件、路由、静态文件）
        self._init_fastapi_middleware()
        self._init_fastapi_routes()
        self._init_fastapi_static()

        self.ws_lock = asyncio.Lock()
        self.ws_connections: Dict[str, WebSocket] = {}

        if self.enable_register:
            self.ros_srv = rospy.Service(
                "register", StringSrv, self._ros_register_callback
            )
            ready_pub = rospy.Publisher("ready", String, queue_size=1, latch=True)
            # 服务准备好后发布
            ready_pub.publish("ready")

            rospy.on_shutdown(self._close)

    def _close(self):
        if self.enable_register and hasattr(self, "ros_srv"):
            self.ros_srv.shutdown()
            rospy.loginfo("register服务关闭")

    def _init_fastapi_middleware(self) -> None:
        """初始化FastAPI中间件：CORS跨域、日志过滤"""
        # 注册CORS中间件（允许所有跨域，保留原有配置）
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        # 注册日志过滤中间件
        self.app.add_middleware(
            FilterLogMiddleware, paths_to_exclude=self.log_exclude_path
        )
        self.app.add_middleware(
            GZipMiddleware,
            minimum_size=1024,  # 仅压缩大于 1KB 的响应（避免小文件压缩得不偿失）
            compresslevel=6,  # 压缩级别 1-9，6 是压缩率和性能的平衡值
        )

    def _init_fastapi_static(self) -> None:
        """挂载静态文件目录（确保目录存在，避免报错）"""
        logger.info(f"home dir: {self.home_dir.absolute()}")
        if self.home_dir.exists():
            self.app.mount(
                "/home",  # 访问路径根目录
                StaticFiles(
                    directory=self.home_dir.absolute(), html=True
                ),  # html=True 启用自动加载index.html
                name="home",  # 给这个挂载点命名，便于区分
            )

        if self.static_dir.exists():
            self.app.mount(
                "/static", StaticFiles(directory=self.static_dir), name="static"
            )
            logger.info(f"FastAPI static directory: {self.static_dir.absolute()}")
        else:
            logger.info(
                f"FastAPI static directory not exist: {self.static_dir}, skip mount"
            )

    def _init_fastapi_routes(self) -> None:
        """初始化FastAPI基础路由：首页、视频流、WebRTC、注册接口、WebSocket"""
        app = self.app

        @app.get("/")
        async def index():
            """首页路由：返回静态index.html"""
            index_path = self.home_dir / "index.html"
            print(index_path)
            return (
                FileResponse(index_path)
                if index_path.exists()
                else JSONResponse({"msg": "index not found"})
            )

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket核心路由：处理WS连接的建立、断开"""
            await websocket.accept()
            async with self.ws_lock:
                conn_id = str(uuid.uuid4())
                self.ws_connections[conn_id] = websocket
            self.trigger("on_connect", {}, conn_id)
            try:
                # 心跳检测（超时无消息则循环，避免永久阻塞）
                while True:
                    json_data = await websocket.receive_json()
                    params = {}
                    if self.router is not None:
                        params["route"] = self.router(json_data)
                    self.trigger("on_message", params, json_data)
            except Exception as e:
                try:
                    async with self.ws_lock:
                        del self.ws_connections[conn_id]
                    self.trigger("on_disconnect", {})
                except:
                    pass

        @app.get("/page_config")
        async def get_page_config():
            return {"status": "success", "msg": BaseUIConfig.generate_json()}

    def _ros_register_callback(self, req: Any) -> Any:
        """ROS register服务回调：动态添加FastAPI路由，与HTTP接口逻辑一致"""
        try:
            route_data = json.loads(req.request)
            path = route_data["path"]
            method = route_data["methods"]
            topic = route_data["endpoint"]
            handler = RequestHandler(path, method, topic)
            route_kwargs = dict2route(route_data)
            route_kwargs.update(dict(endpoint=handler.__call__))
            self.app.add_api_route(**route_kwargs)
            self.app.openapi_schema = None
            self.app.setup()
            logger.info(f"Register FastAPI route from ROS: {method} {path} -> {topic}")
            return StringSrvResponse(
                response=json.dumps({"status": "success", "msg": "OK"})
            )

        except Exception as e:
            logger.error(f"Register FastAPI route from ROS failed: {e}")
            return StringSrvResponse(
                response=json.dumps({"status": "error", "msg": str(e)})
            )

    async def _run_fastapi_server(self, host, port) -> None:
        """异步运行FastAPI服务，内部调用uvicorn，获取异步事件循环"""
        config = Config(self.app, host=host, port=port)
        self.server = uvicorn.Server(config)
        self.loop = asyncio.get_event_loop()  # 保存事件循环，供WS广播使用
        logger.info(f"FastAPI service start: {host}:{port}")

        # 配置uvicorn的日志
        uvicorn_logger_names = ["uvicorn", "uvicorn.access", "uvicorn.error"]
        for logger_name in uvicorn_logger_names:
            uvicorn_logger = logging.getLogger(logger_name)
            # 清空uvicorn自带的Handler
            uvicorn_logger.handlers.clear()
            # 让uvicorn日志传递到根日志器处理
            uvicorn_logger.propagate = True
        await self.server.serve()

    def start_server(self, host: str = "0.0.0.0", port: int = 8000) -> None:
        """启动FastAPI服务（独立线程），与ROS同步逻辑解耦，避免阻塞"""
        if self.server_thread and self.server_thread.is_alive():
            logger.info("FastAPI service is already running! skip start")
            return
        # 创建异步线程运行FastAPI
        self.server_thread = threading.Thread(
            target=asyncio.run,
            args=(self._run_fastapi_server(host, port),),
            daemon=True,  # 守护线程，ROS退出时自动终止
        )
        self.server_thread.start()
        is_ready = self.server_ready.wait(timeout=5.0)
        if not is_ready:
            raise RuntimeError("Server start timeout!")
        logger.info("FastAPI is running!")

    def bind_callback(self, event_name, params, callback):
        if event_name in ["on_post", "on_get"]:
            url = params["url"]
            method = params["method"]
            self.app.add_api_route(url, endpoint=callback, methods=[method])
        else:
            super().bind_callback(event_name, params, callback)

    def send_json(self, conn_id, data: Dict[str, Any]):
        async def worker():
            async with self.ws_lock:
                try:
                    ws = self.ws_connections[conn_id]
                    await ws.send_json(data)
                except Exception as e:
                    logger.error(f"WS Client error: {conn_id}: {e}")
                    del self.ws_connections[conn_id]

        asyncio.run_coroutine_threadsafe(worker(), self.loop)

    def publish(self, data: Dict[str, Any]):
        conn_ids = copy.deepcopy(list(self.ws_connections.keys()))
        for conn_id in conn_ids:
            self.send_json(conn_id, data)
