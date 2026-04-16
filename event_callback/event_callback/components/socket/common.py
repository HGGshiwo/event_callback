from typing import Any

from event_callback.core import BaseEvent, F


class MessageEvent(BaseEvent):
    def __call__(self, route: Any = None):
        kwargs = {}
        if route is not None:
            kwargs["route"] = route
        return super().__call__(**kwargs)
