from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Union

from event_callback.components.http import MessageHandler, MessageType
from event_callback.components.http.ui_config import BaseUIConfig
import uvicorn
import asyncio
import json
import threading
import logging
from pathlib import Path
from uvicorn.config import Config
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from std_msgs.msg import String
from event_callback.utils import dict2route, request2dict, rospy_init_node
from starlette.middleware.gzip import GZipMiddleware

try:
    import rospy
    from std_msgs.msg import Empty
    from event_callback_msg.srv import StringSrv, StringSrvRequest, StringSrvResponse
except:
    pass


# 组件基类导入
from event_callback.core import (
    R,
    BaseComponentHelper,
    BaseComponent,
    BaseConfig,
    CallbackItem,
    CallbackManager,
)
from typing import List
import logging

logger = logging.getLogger(__name__)


class HTTPComponent(BaseComponent):
    """FastAPI桥接组件：基于FastAPI实现HTTP/WS服务，支持与ROS的动态路由映射，
    配置register=True时启用ROS register服务和do_register触发机制
    ## Usage:
    - path: url for callback
    - method: "POST", "GET" etc.
    """

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
            request_json = await request2dict(request)
            ros_resp = self.ros_srv_proxy(request=json.dumps(request_json))
            return json.loads(ros_resp.response)

    class WSManager:
        """WebSocket连接管理器：实现WS连接的增删、线程安全的消息广播，无业务数据处理"""

        def __init__(self, component):
            self.component = component  # 关联父组件，获取事件循环
            self.ws_connections = []
            self.lock = threading.Lock()  # 线程安全锁
            self.messsage_handler_map: Dict[MessageType, MessageHandler] = {}

        async def add_connection(self, ws: WebSocket):
            """添加WS连接，首次发送基础状态数据"""
            data = {}
            with self.lock:
                for handler in self.messsage_handler_map.values():
                    data.update(handler.on_connect())
            await ws.send_json(data)
            with self.lock:
                self.ws_connections.append(ws)

        def remove_connection(self, ws: WebSocket):
            """移除WS连接（线程安全）"""
            with self.lock:
                if ws in self.ws_connections:
                    self.ws_connections.remove(ws)

        def publish(self, data: Dict[str, Any], data_type: MessageType):
            """广播消息到所有WS连接（线程安全，异步非阻塞发送）"""
            with self.lock:
                # 封装消息类型，更新基础数据（保留原有数据结构，无业务逻辑）
                if data_type not in self.messsage_handler_map:
                    self.messsage_handler_map[data_type] = data_type.value()
                message_handler = self.messsage_handler_map[data_type]
                data = message_handler.on_send(data)
                if data is None:
                    return
                # 异步发送消息到每个连接
                for ws in self.ws_connections:
                    asyncio.run_coroutine_threadsafe(
                        self._safe_send(ws, data), self.component.loop
                    )

        async def _safe_send(self, ws: WebSocket, data: dict):
            """安全发送WS消息，异常则自动移除连接"""
            try:
                await ws.send_json(data)
            except Exception:
                self.remove_connection(ws)

    def __init__(self, config: "HTTPConfig"):
        """
        初始化FastAPI-ROS组件

        :param config: 组件配置参数，支持：

            - register: 布尔值，是否启用ROS register服务和do_register触发（默认False）
            - host: FastAPI服务地址（默认0.0.0.0）
            - port: FastAPI服务端口（默认8000）
            - static_dir: 静态文件目录（默认上级目录的static）
            - log_exclude_path: 忽略日志记录的路径
            - websocket_topic: 监听 websocket topic名称
        """
        super().__init__(config)
        # 组件核心配置解析
        self.enable_register = config.register
        self.fastapi_host = config.host
        self.fastapi_port = config.port

        default_static_dir = Path(__file__).parent.parent / "static"
        static_dir = (
            default_static_dir if config.static_dir is None else config.static_dir
        )
        self.static_dir = Path(static_dir)
        default_home_dir = Path(__file__).parents[5] / "event-callback-app" / "dist"
        home_dir = config.home_dir if config.home_dir is not None else default_home_dir
        self.home_dir = Path(home_dir)

        self.websockt_topic = config.websocket_topic
        self.log_exclude_path = config.log_exclude_path
        # ROS相关初始化（确保节点全局唯一，避免重复初始化）
        if self.enable_register:
            self._init_ros_node()
        self.do_register_pub = None  # do_register发布器（register=True时初始化）

        # FastAPI核心实例变量
        self.app = FastAPI()  # FastAPI应用实例
        self.loop = None  # FastAPI异步事件循环
        self.server_thread = None  # FastAPI服务线程（与ROS解耦）
        self.ws_manager = self.WSManager(self)  # WS管理器（关联当前组件）

        # 初始化FastAPI（中间件、路由、静态文件）
        self._init_fastapi_middleware()
        self._init_fastapi_routes()
        self._init_fastapi_static()
        self.start_fastapi_server()

        # 若启用register，初始化ROS register服务和do_register发布器
        if self.enable_register:
            self._init_ros_register_service()
            self.do_register_trigger()
        logger.info(
            f"HTTPComponent register done, register enable: {self.enable_register}"
        )

    def _init_ros_node(self) -> None:
        """初始化ROS节点（全局仅一次，节点名优先取配置，否则用Manager类名）"""
        node_name = self.__class__.__name__.lower()
        rospy_init_node(node_name)
        logger.info(f"HTTPComponent init ros-node: {node_name}")

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
            self.FilterLogMiddleware, paths_to_exclude=self.log_exclude_path
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
        ws_manager = self.ws_manager

        @app.get("/")
        async def index():
            """首页路由：返回静态index.html"""
            index_path = self.home_dir / "index.html"
            return (
                FileResponse(index_path)
                if index_path.exists()
                else JSONResponse({"msg": "index not found"})
            )

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket核心路由：处理WS连接的建立、断开"""
            await websocket.accept()
            await ws_manager.add_connection(websocket)
            try:
                # 心跳检测（超时无消息则循环，避免永久阻塞）
                while True:
                    text = await websocket.receive_text()
                    logger.info(f"ws received: {text}")
            except (WebSocketDisconnect, asyncio.TimeoutError):
                ws_manager.remove_connection(websocket)
            except Exception:
                ws_manager.remove_connection(websocket)

        @app.get("/page_config")
        async def get_page_config():
            return {"status": "success", "msg": BaseUIConfig.generate_json()}

    def _init_ros_register_service(self) -> None:
        """初始化ROS register服务和do_register发布器（enable_register=True时调用）"""
        # 注册ROS /register服务，用于ROS端动态映射FastAPI路由
        rospy.Service("register", StringSrv, self._ros_register_callback)
        logger.info("HTTPComponent register ROS service: /register")
        # 创建do_register发布器，用于触发路由注册（std_msgs/Empty）
        self.do_register_pub = rospy.Publisher(
            "do_register", Empty, queue_size=10, latch=True
        )
        self.ws_sub = rospy.Subscriber(self.websockt_topic, String, self._ws_callback)
        logger.info("HTTPComponent create ROS topic: do_register")

    def _ws_callback(self, data: String):
        json_data = json.loads(data.data)
        data_type: str = json_data.get("type", "state")
        data_type = getattr(MessageType, data_type.upper())
        self.ws_manager.publish(json_data, data_type)

    def _ros_register_callback(self, req: StringSrvRequest) -> StringSrvResponse:
        """ROS register服务回调：动态添加FastAPI路由，与HTTP接口逻辑一致"""
        try:
            route_data = json.loads(req.request)
            path = route_data["path"]
            method = route_data["methods"]
            topic = route_data["endpoint"]
            handler = self.RequestHandler(path, method, topic)
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

    async def _run_fastapi_server(self) -> None:
        """异步运行FastAPI服务，内部调用uvicorn，获取异步事件循环"""
        config = Config(self.app, host=self.fastapi_host, port=self.fastapi_port)
        server = uvicorn.Server(config)
        self.loop = asyncio.get_event_loop()  # 保存事件循环，供WS广播使用
        logger.info(f"FastAPI service start: {self.fastapi_host}:{self.fastapi_port}")
        
        # 配置uvicorn的日志
        uvicorn_logger_names = ["uvicorn", "uvicorn.access", "uvicorn.error"]
        for logger_name in uvicorn_logger_names:
            uvicorn_logger = logging.getLogger(logger_name)
            # 清空uvicorn自带的Handler
            uvicorn_logger.handlers.clear()
            # 让uvicorn日志传递到根日志器处理
            uvicorn_logger.propagate = True
        await server.serve()

    def start_fastapi_server(self) -> None:
        """启动FastAPI服务（独立线程），与ROS同步逻辑解耦，避免阻塞"""
        if self.server_thread and self.server_thread.is_alive():
            logger.info("FastAPI service is already running! skip start")
            return
        # 创建异步线程运行FastAPI
        self.server_thread = threading.Thread(
            target=asyncio.run,
            args=(self._run_fastapi_server(),),
            daemon=True,  # 守护线程，ROS退出时自动终止
        )
        self.server_thread.start()
        logger.info("FastAPI is running!")

    def do_register_trigger(self) -> None:
        """触发do_register发布：向ROS do_register话题发布Empty消息，供外部调用"""
        if not self.enable_register or not self.do_register_pub:
            logger.info("register is disable, can't trigger do_register")
            return
        self.do_register_pub.publish(Empty())
        logger.info("HTTPComponent trigger do_register")

    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        """实现BaseComponent抽象方法：组件核心回调/服务注册入口
        基类强制要求实现，此处作为FastAPI服务启动和ROS注册的统一入口
        """
        for callback, args, kwargs in callbacks:
            # 不用partial是怕丢失__name__等meta信息
            # endpoint = MethodType(callback, self.manager_instance)
            endpoint = callback
            methods = kwargs.get("methods", [])
            methods = methods + list(args[1:])
            self.app.add_api_route(args[0], endpoint=endpoint, methods=methods)


@dataclass
class HTTPConfig(BaseConfig):
    target: type = field(init=False, default=HTTPComponent)
    host: str = "0.0.0.0"
    port: int = 8000
    register: bool = False
    static_dir: Union[str, Path, None] = None
    home_dir: Union[str, Path, None] = None
    websocket_topic: str = "ws"
    log_exclude_path: List = field(default_factory=list)


class http(BaseComponentHelper):
    target = HTTPComponent

    @classmethod
    def _base(cls, url: str, method: str, frequency: float):
        return R._create_comp_decorator(cls.target, url, method, frequency=frequency)

    @classmethod
    def post(cls, url: str, frequency: Optional[float] = None):
        return cls._base(url, "POST", frequency=frequency)

    @classmethod
    def get(cls, url: str, frequency: Optional[float] = None):
        return cls._base(url, "GET", frequency=frequency)

    @classmethod
    def ws_send(
        cls,
        manager_instance: CallbackManager,
        json: Dict,
        data_type: Optional[MessageType] = None,
    ):
        if data_type is None:
            data_type = MessageType.STATE
        comp = manager_instance.get_component_instance(cls.target)
        if comp is None:  # 组件没有初始化完成，不允许调用
            return
        return comp.ws_manager.publish(json, data_type)
