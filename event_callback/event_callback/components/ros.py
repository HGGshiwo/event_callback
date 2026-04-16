from typing import Optional, Type

from event_callback.core import BaseComponent, BaseEvent, F


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

    def bind_callback(self, event_name, params, callback):
        assert event_name == "on_topic"
        import rospy

        sub = rospy.Subscriber(
            params["topic_name"],
            params["topic_type"],
            callback,
            queue_size=params["queue_size"],
        )
        # 引用防止被清理
        self.sub.append(sub)
