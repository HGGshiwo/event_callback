import asyncio
import inspect
import json
import logging
import traceback
from concurrent.futures import ThreadPoolExecutor
from contextlib import AsyncExitStack
from functools import partial
from threading import Thread
from typing import Any, Callable

from std_msgs.msg import String

from event_callback.components.http.core import GETEvent, MessageEvent, POSTEvent
from event_callback.core import BaseEvent

try:
    from typing import TypeAlias
except:
    from typing_extensions import TypeAlias

from fastapi.dependencies.models import Dependant
from fastapi.dependencies.utils import get_dependant, solve_dependencies

from event_callback.core import BaseComponent
from event_callback.ros_utils import rospy_is_shutdown

from .utils import dict2request, route2dict

try:
    import rospy
    from event_callback_msg.srv import StringSrv, StringSrvRequest, StringSrvResponse
except:
    StringSrvRequest: TypeAlias = Any
    StringSrvResponse: TypeAlias = Any
    StringSrv: TypeAlias = Any


logger = logging.getLogger(__name__)


class HTTP_ProxyComponent(BaseComponent):
    """ROS Http代理组件：负责ROS Service回调注册、Http-ROS映射、触发订阅处理，实现BaseComponent抽象方法"""

    on_post = POSTEvent()
    on_get = GETEvent()
    on_connect = BaseEvent()
    on_disconnect = BaseEvent()
    on_message = MessageEvent()
    on_ready = BaseEvent()  # http服务器已经就绪

    def __init__(self):
        super().__init__()
        self._callbacks = None  # 记录下需要注册的路由
        self._excutor = ThreadPoolExecutor(thread_name_prefix="http_ros_proxy_")
        self._event_loop = None
        self._thread = Thread(target=lambda: asyncio.run(self._run()), daemon=True)
        self._thread.start()

    def bind_done(self):
        # 必须在bind完成后订阅，触发注册，不然_route是空的
        self.ready_sub = rospy.Subscriber(
            "ready", String, callback=self._init_ros_srv_config
        )

    async def _run(self):
        self._event_loop = asyncio.get_event_loop()
        while not rospy_is_shutdown():
            await asyncio.sleep(0.001)

    def _init_ros_srv_config(self, data) -> None:
        """初始化ROS服务配置：等待注册服务、创建服务代理"""
        # 等待register服务上线（超时会自动抛出异常，符合ROS服务调用规范）
        rospy.wait_for_service("register", timeout=100)
        rospy.loginfo("reigster is OK")
        # 创建register服务代理，用于Http-ROS服务映射注册
        self.register_srv = rospy.ServiceProxy("register", StringSrv)
        for event_name, params, callback in self._routes:
            if event_name in ["on_post", "on_get"]:
                url = params["url"]
                method = params["method"]
                self._register_ros_services(callback, url, method)
            else:
                pass  # 不是http的回调
        self.trigger("on_ready", {})

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
                status="error",
                msg=str(e),
                detail=traceback.format_exc(),
            )
        finally:
            await async_exit_stack.aclose()
            return StringSrvResponse(response=json.dumps(response))

    def _register_ros_services(self, func: Callable, path: str, method: str) -> None:
        """从Manager中读取路由回调，完成ROS Service注册及Http-ROS映射"""
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

    def publish(self, data):
        raise NotImplementedError()
