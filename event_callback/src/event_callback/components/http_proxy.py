from event_callback.core import (
    R,
    BaseComponentHelper,
    BaseComponent,
    CallbackItem,
    CallbackManager,
)
from typing import Dict, Any, List, Never, Type

import json, traceback

try:
    import rospy
    from event_callback_msg.srv import Register, ProcessRequest, ProcessRequestResponse
    from std_msgs.msg import Empty
except:

    class ProcessRequest:  # type: ignore
        pass

    class ProcessRequestResponse:
        pass


class HTTP_ProxyComponent(BaseComponent):
    """ROS Http代理组件：负责ROS Service回调注册、Http-ROS映射、触发订阅处理，实现BaseComponent抽象方法"""

    def __init__(self, **config: Dict[str, Any]):
        super().__init__(**config)

        self._init_ros_node()
        self._srv_registered = False  # Service注册状态标记，防止重复注册
        self._init_ros_srv_config()  # 初始化ROS服务相关配置
        self._register_trigger_sub()  # 注册Service触发的订阅话题
        self._callbacks = None  # 记录下需要注册的路由

    def _init_ros_node(self) -> None:
        """初始化ROS节点，优先使用配置中的节点名，无配置则使用Manager类名小写"""
        node_name = self.config.get("node_name", self.__class__.__name__.lower())
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=False)

    def _init_ros_srv_config(self) -> None:
        """初始化ROS服务配置：等待注册服务、创建服务代理"""
        # 等待register服务上线（超时会自动抛出异常，符合ROS服务调用规范）
        rospy.wait_for_service("register")
        # 创建register服务代理，用于Http-ROS服务映射注册
        self.register_srv = rospy.ServiceProxy("register", Register)

    def _register_trigger_sub(self) -> None:
        """注册触发Service注册的Topic：收到空消息则执行Service注册"""
        rospy.Subscriber("do_register", Empty, self._on_trigger_srv_register)

    def _on_trigger_srv_register(self, _: Any = None) -> None:
        """do_register Topic回调：触发ROS Service注册，已注册则直接返回"""
        if self._srv_registered:
            return
        self._register_ros_services()
        self._srv_registered = True

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

            def _srv_cb(data: ProcessRequest, func=func):
                """ROS Service回调函数：解析JSON请求，执行业务逻辑，返回JSON响应"""
                try:
                    # 解析请求体（ROS服务传参为JSON字符串）
                    req = json.loads(data.request)
                    # 执行Manager绑定的路由回调，传参并获取结果
                    res = func(**req)
                    # 构造成功响应（ROS服务响应为JSON字符串）
                    return ProcessRequestResponse(response=json.dumps(res))
                except Exception as e:
                    # 构造异常响应，包含错误信息和堆栈详情
                    return ProcessRequestResponse(
                        response=json.dumps(
                            {
                                "status": "error",
                                "msg": str(e),
                                "detail": traceback.format_exc(),
                            }
                        )
                    )

            # 注册ROS Service，捕获已存在异常（避免重复注册报错）
            try:
                rospy.Service(srv_name, ProcessRequest, _srv_cb)
            except rospy.service.ServiceException:
                pass

            # 调用register服务完成Http-ROS服务映射，映射失败则抛出异常
            reg_res = self.register_srv(path=path, method=method, topic=srv_name)
            if json.loads(reg_res.response)["status"] != "success":
                raise ValueError(f"ROS HTTP mapping failed: {path} -> {srv_name}")

    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        if self._callbacks is None:
            self._callbacks = callbacks
        else:
            self._callbacks.extend(callbacks)


class http_porxy(BaseComponentHelper):
    target = HTTP_ProxyComponent

    @classmethod
    def post(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "POST")

    @classmethod
    def get(cls, url: str):
        return R._create_comp_decorator(cls.target, url, "GET")

    @classmethod
    def config(cls):
        return cls.target, {}
