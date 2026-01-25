# 暴露核心公共类
from event_callback.core import BaseComponent, CallbackManager, R
# 暴露各组件（新增组件后，在此处添加即可）
from event_callback.components.ros import ROSComponent
from event_callback.components.http import HTTPComponent
from event_callback.components.ros_httproxy import ROSHTTProxyComponent
from event_callback.components.socket import SocketClientComponent, SocketServerComponent

# 包版本、作者等信息（可选）
__version__ = "1.0.0"
__author__ = "event-callback"