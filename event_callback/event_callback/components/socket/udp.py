import logging
import selectors
import socket
import threading
from typing import Callable

from event_callback.core import BaseComponent

from .common import MessageEvent

logger = logging.getLogger(__name__)


class UDPComponent(BaseComponent):
    # 事件声明：因为没有连接状态，只有纯粹的消息事件
    # 回调签名：Manager实例, data字节, address元组(ip, port)
    on_message = MessageEvent()

    def __init__(self, router: Callable = None, enable_broadcast: bool = False):
        super().__init__()
        self.router = router
        self.enable_broadcast = enable_broadcast

        self._socket = None
        self._running = False
        self._sel = selectors.DefaultSelector()

    def start_server(self, host: str = "0.0.0.0", port: int = 0):
        """启动 UDP 节点"""
        if self._running:
            return

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 开启广播权限
        if self.enable_broadcast:
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # 绑定端口（如果 port=0，系统会自动分配可用端口）
        self._socket.bind((host, port))

        # 获取实际绑定的端口
        actual_port = self._socket.getsockname()[1]
        logger.info(f"[UdpComponent] 已启动，绑定于 {host}:{actual_port}")
        self._running = True

        # 将 socket 注册到 selector，监听可读事件
        self._socket.setblocking(False)
        self._sel.register(self._socket, selectors.EVENT_READ)
        # 启动后台守护线程进行网络循环
        threading.Thread(target=self._network_loop, daemon=True).start()

    def _network_loop(self):
        """底层的 selector 循环，与 TCP 一致的优雅架构"""
        while self._running:
            # timeout=1.0 保证线程能够及时响应 _running=False 的退出指令
            events = self._sel.select(timeout=1.0)
            for key, mask in events:
                try:
                    # UDP 接收特征：用 recvfrom，最大包一般限制在 65535 字节以内
                    data, addr = self._socket.recvfrom(65535)
                    if data:
                        # 把消息和来源地址抛给业务层
                        params = {}
                        if self.router is not None:
                            params["route"] = self.router(data)
                        self.trigger("on_message", params, data, addr)
                except OSError:
                    pass

    def send(self, data: bytes, target_host: str, target_port: int):
        """直接指定目标 IP 和端口发送"""
        if self._socket:
            self._socket.sendto(data, (target_host, target_port))

    def broadcast(self, data: bytes, target_port: int):
        """局域网广播（需要初始化时 enable_broadcast=True）"""
        if self._socket and self.enable_broadcast:
            # 255.255.255.255 是全局广播地址
            self._socket.sendto(data, ("255.255.255.255", target_port))

    def stop(self):
        """优雅关闭"""
        self._running = False
        if self._socket:
            self._sel.unregister(self._socket)
            self._socket.close()
