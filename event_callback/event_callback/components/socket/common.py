from typing import Any

from event_callback.core import BaseEvent, F


class MessageEvent(BaseEvent):
    def __call__(self, route: Any = None):
        def decorator(func: F):
            kwargs = {}
            if route is not None:
                kwargs["route"] = route
            return self._mark_method(func, **kwargs)

        return decorator
