from functools import partial
from typing import Any, Dict, Optional, Type
try:
    import rospy
except:
    pass
from event_callback.core import R, BaseComponent, BaseComponentHelper, CallbackManager


class ROSComponent(BaseComponent):
    """ROS Topic组件：仅负责ROS Topic回调的注册与处理，实现BaseComponent抽象方法"""

    def __init__(self, manager_instance: CallbackManager, **config: Dict[str, Any]):
        super().__init__(manager_instance, **config)
        # 初始化ROS节点（确保全局仅初始化一次，避免重复初始化报错）
        self._init_ros_node()

    def _init_ros_node(self) -> None:
        """初始化ROS节点，优先使用配置中的节点名，无配置则使用Manager类名小写"""
        node_name = self.config.get(
            "node_name", self.manager_instance.__class__.__name__.lower()
        )
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=False)

    def register_callbacks(self, callbacks) -> None:
        """从Manager的回调注册表中读取Topic回调，完成ROS Topic订阅注册"""
        # 获取当前组件绑定的所有Topic回调参数
        for callback, args, kwargs in callbacks:
            # 注册Topic订阅，将Manager实例绑定到回调函数（偏函数传参）
            topic_name, topic_type, queue_size = args
            callback = partial(callback, self.manager_instance)
            rospy.Subscriber(
                topic_name,
                topic_type,
                callback,
                queue_size=queue_size,
            )


class ros(BaseComponentHelper):
    target = ROSComponent

    @classmethod
    def topic(cls, topic_name: str, topic_type: Type, queue_size: Optional[int] = None):
        R._create_comp_decorator(cls.target, topic_name, topic_type, queue_size)

    @classmethod
    def config(cls):
        return cls.target, {}
