from types import MethodType
from typing import Any, Dict, Optional, Union

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

try:
    import rospy
except:
    pass

# ROS相关消息/服务导入（仅保留框架必要的，删除业务相关）
from std_msgs.msg import Empty
from event_callback_msg.srv import (
    ProcessRequest,
    Register,
    RegisterResponse,
    RegisterRequest,
)

# 组件基类导入（与之前的ROS组件保持一致）
from event_callback.core import R, BaseComponentHelper, BaseComponent, CallbackItem, CallbackManager
from typing import List


class HTTPComponent(BaseComponent):
    """FastAPI桥接组件：基于FastAPI实现HTTP/WS服务，支持与ROS的动态路由映射，
    配置register=True时启用ROS register服务和do_register触发机制
    ## Usage:
    - path: url for callback
    - method: "POST", "GET" etc.
    """

    class FilterLogMiddleware(BaseHTTPMiddleware):
        """FastAPI日志过滤中间件：屏蔽指定路径的访问日志，减少冗余输出"""

        async def dispatch(self, request: Request, call_next):
            # 需过滤的路径前缀，保留原有配置
            paths_to_exclude = ["static", "get_gps", "get_gpsv2"]
            logger = logging.getLogger("uvicorn.access")
            logger.disabled = any(p in request.url.path for p in paths_to_exclude)
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
            self.ros_srv_proxy = rospy.ServiceProxy(topic, ProcessRequest)

        async def __call__(self, request: Request):
            """解析FastAPI请求（路径/查询/体参数），调用ROS服务并返回JSON响应"""
            request_data = {}
            # 合并路径参数、查询参数
            request_data.update(request.path_params)
            request_data.update(request.query_params)
            # 合并请求体（兼容无body/非JSON格式）
            try:
                body = await request.json()
                request_data.update(body)
            except Exception:
                pass
            # 调用ROS服务并解析响应
            ros_resp = self.ros_srv_proxy(request=json.dumps(request_data))
            return json.loads(ros_resp.response)

    class WSManager:
        """WebSocket连接管理器：实现WS连接的增删、线程安全的消息广播，无业务数据处理"""

        def __init__(self, component):
            self.component = component  # 关联父组件，获取事件循环
            self.ws_connections = []
            self.base_data = {
                "type": "state",
                "connected": False,
                "record": False,
                "event": [],
            }
            self.lock = threading.Lock()  # 线程安全锁

        async def add_connection(self, ws: WebSocket):
            """添加WS连接，首次发送基础状态数据"""
            await ws.send_json(self.base_data)
            with self.lock:
                self.ws_connections.append(ws)

        def remove_connection(self, ws: WebSocket):
            """移除WS连接（线程安全）"""
            with self.lock:
                if ws in self.ws_connections:
                    self.ws_connections.remove(ws)

        def publish(self, data: dict, data_type: str = "state"):
            """广播消息到所有WS连接（线程安全，异步非阻塞发送）"""
            with self.lock:
                # 封装消息类型，更新基础数据（保留原有数据结构，无业务逻辑）
                send_data = {**data, "type": data_type}
                if data_type == "state":
                    self.base_data.update(send_data)
                elif data_type == "event":
                    self.base_data["event"].append(send_data)
                # 复制连接列表，避免长时间持有锁
                ws_copy = self.ws_connections.copy()

            # 异步发送消息到每个连接
            for ws in ws_copy:
                asyncio.run_coroutine_threadsafe(
                    self._safe_send(ws, send_data), self.component.loop
                )

        async def _safe_send(self, ws: WebSocket, data: dict):
            """安全发送WS消息，异常则自动移除连接"""
            try:
                await ws.send_json(data)
            except Exception:
                self.remove_connection(ws)

    def __init__(self, manager_instance: CallbackManager, **config: Dict[str, Any]):
        """
        初始化FastAPI-ROS组件
        :param manager_instance: BaseManager子类实例（组件基类要求）
        :param config: 组件配置参数，支持：
            - register: 布尔值，是否启用ROS register服务和do_register触发（默认False）
            - host: FastAPI服务地址（默认0.0.0.0）
            - port: FastAPI服务端口（默认8000）
            - static_dir: 静态文件目录（默认上级目录的static）
        """
        super().__init__(manager_instance, **config)
        # 组件核心配置解析
        self.enable_register = self.config.get("register", False)
        self.fastapi_host = self.config.get("host", "0.0.0.0")
        self.fastapi_port = self.config.get("port", 8000)
        self.static_dir = Path(
            self.config.get("static_dir", Path(__file__).parent.parent / "static")
        )

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

        # 若启用register，初始化ROS register服务和do_register发布器
        if self.enable_register:
            self._init_ros_register_service()
        self.start_fastapi_server()
        print(f"FastapiRosComponent组件完成回调注册，register功能: {self.enable_register}")

    def _init_ros_node(self) -> None:
        """初始化ROS节点（全局仅一次，节点名优先取配置，否则用Manager类名）"""
        node_name = self.config.get(
            "node_name", self.manager_instance.__class__.__name__.lower() + "_fastapi"
        )
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=False)
            print(f"Http组件初始化ROS节点: {node_name}")

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
        self.app.add_middleware(self.FilterLogMiddleware)

    def _init_fastapi_static(self) -> None:
        """挂载静态文件目录（确保目录存在，避免报错）"""
        if self.static_dir.exists():
            self.app.mount(
                "/static", StaticFiles(directory=self.static_dir), name="static"
            )
        else:
            print(f"FastAPI静态文件目录不存在: {self.static_dir}，跳过挂载")

    def _init_fastapi_routes(self) -> None:
        """初始化FastAPI基础路由：首页、视频流、WebRTC、注册接口、WebSocket"""
        app = self.app
        ws_manager = self.ws_manager

        @app.get("/")
        async def index():
            """首页路由：返回静态index.html"""
            index_path = self.static_dir / "index.html"
            return (
                FileResponse(index_path)
                if index_path.exists()
                else JSONResponse({"msg": "index not found"})
            )

        @app.post("/register")
        async def http_register_route(request: Request):
            """HTTP注册接口：动态添加FastAPI路由，映射到ROS Service"""
            data = await request.json()
            path, method, topic = data["path"], data["method"], data["topic"]
            # 创建请求处理器并动态添加路由
            handler = self.RequestHandler(path, method, topic)
            app.add_api_route(path, handler.__call__, methods=[method])
            # 刷新OpenAPI文档，使动态路由生效
            app.openapi_schema = None
            app.setup()
            return {"status": "success", "msg": "OK"}

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket核心路由：处理WS连接的建立、断开"""
            await websocket.accept()
            await ws_manager.add_connection(websocket)
            try:
                # 心跳检测（超时无消息则循环，避免永久阻塞）
                while True:
                    await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
            except (WebSocketDisconnect, asyncio.TimeoutError):
                ws_manager.remove_connection(websocket)
            except Exception:
                ws_manager.remove_connection(websocket)

    def _init_ros_register_service(self) -> None:
        """初始化ROS register服务和do_register发布器（enable_register=True时调用）"""
        # 注册ROS /register服务，用于ROS端动态映射FastAPI路由
        rospy.Service("register", Register, self._ros_register_callback)
        print("FastAPI-ROS组件注册ROS服务: /register")
        # 创建do_register发布器，用于触发路由注册（std_msgs/Empty）
        self.do_register_pub = rospy.Publisher(
            "do_register", Empty, queue_size=10, latch=True
        )
        print("FastAPI-ROS组件创建ROS发布器: do_register")

    def _ros_register_callback(self, req: RegisterRequest) -> RegisterResponse:
        """ROS register服务回调：动态添加FastAPI路由，与HTTP接口逻辑一致"""
        try:
            path, method, topic = req.path, req.method, req.topic
            handler = self.RequestHandler(path, method, topic)
            self.app.add_api_route(path, handler.__call__, methods=[method])
            self.app.openapi_schema = None
            self.app.setup()
            print(f"ROS端动态注册FastAPI路由: {method} {path} -> {topic}")
            return RegisterResponse(
                response=json.dumps({"status": "success", "msg": "OK"})
            )

        except Exception as e:
            print(f"ROS注册路由失败: {e}")
            return RegisterResponse(
                response=json.dumps({"status": "error", "msg": str(e)})
            )

    async def _run_fastapi_server(self) -> None:
        """异步运行FastAPI服务，内部调用uvicorn，获取异步事件循环"""
        config = Config(self.app, host=self.fastapi_host, port=self.fastapi_port)
        server = uvicorn.Server(config)
        self.loop = asyncio.get_event_loop()  # 保存事件循环，供WS广播使用
        print(f"FastAPI服务启动: {self.fastapi_host}:{self.fastapi_port}")
        await server.serve()

    def start_fastapi_server(self) -> None:
        """启动FastAPI服务（独立线程），与ROS同步逻辑解耦，避免阻塞"""
        if self.server_thread and self.server_thread.is_alive():
            print("FastAPI服务已在运行，无需重复启动")
            return
        # 创建异步线程运行FastAPI
        self.server_thread = threading.Thread(
            target=asyncio.run,
            args=(self._run_fastapi_server(),),
            daemon=True,  # 守护线程，ROS退出时自动终止
        )
        self.server_thread.start()
        print("FastAPI服务线程已启动")

    def do_register_trigger(self) -> None:
        """触发do_register发布：向ROS do_register话题发布Empty消息，供外部调用"""
        if not self.enable_register or not self.do_register_pub:
            print("未启用register功能，无法触发do_register")
            return
        self.do_register_pub.publish(Empty())
        print("FastAPI-ROS组件触发do_register发布")

    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        """实现BaseComponent抽象方法：组件核心回调/服务注册入口
        基类强制要求实现，此处作为FastAPI服务启动和ROS注册的统一入口
        """
        for callback, args, kwargs in callbacks:
            # 不用partial是怕丢失__name__等meta信息
            endpoint = MethodType(callback, self.manager_instance)
            methods = kwargs.get("methods", [])
            methods = methods + list(args[1:])
            self.app.add_api_route(args[0], endpoint=endpoint, methods=methods)


class http(BaseComponentHelper):
    target = HTTPComponent

    @classmethod
    def post(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "POST")

    @classmethod
    def get(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "GET")

    @classmethod
    def config(
        cls,
        host: Optional[str] = "0.0.0.0",
        port: Optional[int] = 8000,
        register: Optional[bool] = False,
        static_dir: Optional[Union[str, Path]] = None,
    ):
        kwargs = {
            "host": host,
            "port": port,
            "register": register,
            "static_idr": static_dir,
        }
        return cls.target, kwargs
