# 暴露核心公共类
from event_callback.core import BaseComponent, CallbackManager, R

# 暴露各组件（新增组件后，在此处添加即可）
from event_callback.components.ros import ros
from event_callback.components.http import http
from event_callback.components.http_proxy import http_porxy
from event_callback.components.socket import socketc, sockets

# 包版本、作者等信息（可选）
__version__ = "1.0.0"
__author__ = "event-callback"
