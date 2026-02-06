import copy
import datetime
from typing import Any, Dict

from event_callback.utils import BaseEnum


class MessageHandler:
    def on_connect(self):
        return {}

    def on_send(self, data: Dict[str, Any]):
        return data


class StateMessageHandler(MessageHandler):
    def __init__(self):
        super().__init__()
        self.state = {}

    def on_send(self, data: Dict[str, Any]):
        self.state.update(**copy.deepcopy(data))
        data["type"] = "state"
        return data

    def on_connect(self):
        return {"type": "state", **copy.deepcopy(self.state)}


class EventMessageHandler(MessageHandler):
    name = "event"

    def __init__(self):
        super().__init__()
        self.state = []

    def on_send(self, data: Dict[str, Any]):
        """为消息添加时间戳"""
        timestamp = int(1000 * datetime.datetime.now().timestamp())
        data["timestamp"] = timestamp
        data["type"] = self.name
        self.state.append(copy.deepcopy(data))
        return data

    def on_connect(self):
        return {self.name: copy.deepcopy(self.state), "type": "state"}

    @staticmethod
    def create(name: str):
        _name = name

        class NamedEventMessageHandler(EventMessageHandler):
            name = _name

        return NamedEventMessageHandler


class MessageType(BaseEnum):
    STATE = StateMessageHandler
    ERROR = EventMessageHandler.create("error")
    WARN = EventMessageHandler.create("warn")
    INFO = EventMessageHandler.create("info")
    DEBUG = EventMessageHandler.create("debug")
