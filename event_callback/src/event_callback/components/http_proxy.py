import asyncio
from contextlib import AsyncExitStack
from dataclasses import dataclass, field
from functools import partial
import inspect
from threading import Thread

from event_callback.core import (
    R,
    BaseComponentHelper,
    BaseComponent,
    CallbackItem,
)
from typing import Callable, Dict, Any, List
from fastapi.dependencies.utils import solve_dependencies, get_dependant
from fastapi.dependencies.models import Dependant
import json, traceback
from concurrent.futures import ThreadPoolExecutor
import logging
from event_callback.utils import (
    dict2request,
    rospy_init_node,
    rospy_is_shutdown,
    route2dict,
)
from event_callback_msg.srv import StringSrvRequest, StringSrvResponse, StringSrv

try:
    import rospy
    from std_msgs.msg import Empty
except:

    class ProcessRequest:  # type: ignore
        pass

    class ProcessRequestResponse:
        pass


logger = logging.getLogger(__name__)


class HTTP_ProxyComponent(BaseComponent):
    """ROS Http代理组件：负责ROS Service回调注册、Http-ROS映射、触发订阅处理，实现BaseComponent抽象方法"""

    def __init__(self, config: "HTTP_ProxyConfig"):
        super().__init__(config)

        self._init_ros_node()
        self._receive_register = False  # 是否收到了register请求，收到需要注册
        self._init_ros_srv_config()  # 初始化ROS服务相关配置
        self._register_trigger_sub()  # 注册Service触发的订阅话题
        self._callbacks = None  # 记录下需要注册的路由
        self._excutor = ThreadPoolExecutor(thread_name_prefix="http_ros_proxy_")
        self._event_loop = None
        self._thread = Thread(target=lambda: asyncio.run(self._run()), daemon=True)
        self._thread.start()
        
    async def _run(self):
        self._event_loop = asyncio.get_event_loop()
        while not rospy_is_shutdown():
            await asyncio.sleep(0.001)

    def _init_ros_node(self) -> None:
        """初始化ROS节点，优先使用配置中的节点名，无配置则使用Manager类名小写"""
        node_name = self.__class__.__name__.lower()
        rospy_init_node(node_name)

    def _init_ros_srv_config(self) -> None:
        """初始化ROS服务配置：等待注册服务、创建服务代理"""
        # 等待register服务上线（超时会自动抛出异常，符合ROS服务调用规范）
        rospy.wait_for_service("register")
        # 创建register服务代理，用于Http-ROS服务映射注册
        self.register_srv = rospy.ServiceProxy("register", StringSrv)

    def _register_trigger_sub(self) -> None:
        """注册触发Service注册的Topic：收到空消息则执行Service注册"""

        rospy.Subscriber("do_register", Empty, self._on_trigger_srv_register)

    def _on_trigger_srv_register(self, _: Any = None) -> None:
        """do_register Topic回调：触发ROS Service注册，已注册则直接返回"""
        if self._callbacks is None:
            self._receive_register = True
            return
        self._register_ros_services()

    def _async_run(self, func: Callable):
        """将一个异步函数包装为同步函数"""

        def sync_func(*args, **kwargs):
            future = asyncio.run_coroutine_threadsafe(
                func(*args, **kwargs), self._event_loop
            )
            try:
                result = future.result()
                return result
            except Exception as e:
                logger.exception(str(e))

        return sync_func

    @staticmethod
    async def _srv_cb(data: StringSrvRequest, func: Callable, dependant: Dependant):
        """ROS Service回调函数：解析JSON请求，执行业务逻辑，返回JSON响应"""
        async_exit_stack = AsyncExitStack()
        try:
            request_data = json.loads(data.request)
            request = await dict2request(request_data)
            try:
                body = json.loads(request_data["body"])
            except:
                body = {}
            solved_result = await solve_dependencies(
                request=request,
                dependant=dependant,
                body=body,
                embed_body_fields=False,
                async_exit_stack=async_exit_stack,
            )
            if solved_result.errors:
                raise ValueError(solved_result.errors)
            if inspect.iscoroutinefunction(func):
                res = await func(**solved_result.values)
            else:
                res = func(**solved_result.values)
            # 构造成功响应（ROS服务响应为JSON字符串）
            response = res
        except Exception as e:
            # 构造异常响应，包含错误信息和堆栈详情
            response = dict(
                states="error",
                msg=str(e),
                detail=traceback.format_exc(),
            )
        finally:
            await async_exit_stack.aclose()
            return StringSrvResponse(response=json.dumps(response))

    def _register_ros_services(self) -> None:
        """从Manager中读取路由回调，完成ROS Service注册及Http-ROS映射"""
        if self._callbacks is None:
            return
        for func, args, kwargs in self._callbacks:
            path, method = args

            # 处理Service名称：去除路径参数/查询参数，替换斜杠为下划线，保证ROS服务名规范
            srv_name = path.split("{")[0].split("?")[0].replace("/", "_")
            if not srv_name.startswith("/"):
                srv_name = f"/{srv_name}"

            dependant = get_dependant(path=path, call=func)

            # 注册ROS Service，捕获已存在异常（避免重复注册报错）
            try:
                _srv_cb = partial(
                    HTTP_ProxyComponent._srv_cb, func=func, dependant=dependant
                )
                handler = self._async_run(_srv_cb)
                rospy.Service(srv_name, StringSrv, handler)
            except rospy.service.ServiceException:
                pass

            # 调用register服务完成Http-ROS服务映射，映射失败则抛出异常
            route_data = route2dict(url=path, method=method, func=func)
            route_data["endpoint"] = srv_name  # 添加topic对应名称
            reg_res = self.register_srv(json.dumps(route_data))
            if json.loads(reg_res.response)["status"] != "success":
                raise ValueError(f"ROS HTTP mapping failed: {path} -> {srv_name}")

    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        if self._callbacks is None:
            self._callbacks = callbacks
        else:
            self._callbacks.extend(callbacks)
        if self._receive_register:
            self._register_ros_services()
            self._receive_register = False


@dataclass
class HTTP_ProxyConfig:
    target: type = field(init=False, default=HTTP_ProxyComponent)


class http_proxy(BaseComponentHelper):
    target = HTTP_ProxyComponent

    @classmethod
    def post(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "POST")

    @classmethod
    def get(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "GET")
