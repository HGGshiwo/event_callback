import json
from dataclasses import dataclass, field
from typing import Callable, Optional, Type

try:
    import rospy
except:
    pass
from std_msgs.msg import String

from event_callback.core import BaseComponent, BaseComponentHelper, BaseConfig, R


class ROSComponent(BaseComponent):
    """ROS Topic组件：仅负责ROS Topic回调的注册与处理，实现BaseComponent抽象方法"""

    def __init__(self, config: "ROSConfig"):
        super().__init__(config)
        # 初始化ROS节点（确保全局仅初始化一次，避免重复初始化报错）
        # self._init_ros_node()

    # def _init_ros_node(self) -> None:
    #     """初始化ROS节点，优先使用配置中的节点名，无配置则使用Manager类名小写"""
    #     node_name = self.__class__.__name__.lower()
    #     rospy_init_node(node_name)


@dataclass
class ROSConfig(BaseConfig):
    target: type = field(init=False, default=ROSComponent)


class ros(BaseComponentHelper):
    target = ROSComponent

    @staticmethod
    def create_wsproxy():
        class WSProxy:
            def __init__(self):
                self.pub = rospy.Publisher("/mavproxy/ws", String, queue_size=1)

            def _send(self, data: dict):
                self.pub.publish(json.dumps(data))

            def info(self, data: dict):
                self._send(dict(info=data, type="info"))

            def error(self, data: dict):
                self._send(dict(error=data, type="error"))

            def state(self, data: dict):
                data["type"] = "state"
                self._send(data)

        return WSProxy()

    @classmethod
    def topic(
        cls,
        topic_name: str,
        topic_type: Type,
        queue_size: Optional[int] = None,
        frequency: Optional[float] = None,
    ):
        def register_callback(self: ROSComponent, callback: Callable):
            rospy.Subscriber(
                topic_name,
                topic_type,
                callback,
                queue_size=queue_size,
            )

        return R._create_comp_decorator(
            cls.target, register_callback, frequency=frequency
        )
