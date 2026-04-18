import time
from typing import Optional, Type

import rospy

from event_callback.core import BaseComponent, BaseEvent


class ROSTopicEvent(BaseEvent):
    def __call__(
        self,
        topic_name: str,
        topic_type: Type,
        queue_size: Optional[int] = None,
    ):
        return super().__call__(
            topic_name=topic_name, topic_type=topic_type, queue_size=queue_size
        )


class ROSComponent(BaseComponent):
    on_topic = ROSTopicEvent()

    def __init__(self):
        super().__init__()
        self.sub = []

    @staticmethod
    def create_publisher(topic_name, topic_type, queue_size=None):
        pub = rospy.Publisher(topic_name, topic_type, queue_size=queue_size)

        def publish(data):
            pub.publish(data)
            time.sleep(0.001)  # 让rospy有机会得到GIL

        return publish

    def bind_callback(self, event_name, params, callback):
        assert event_name == "on_topic"
        import rospy

        super().bind_callback(event_name, params, callback)

        sub = rospy.Subscriber(
            params["topic_name"],
            params["topic_type"],
            lambda *args, **kwargs: self.trigger(event_name, params, *args, **kwargs),
            queue_size=params["queue_size"],
        )
        # 引用防止被清理
        self.sub.append(sub)
