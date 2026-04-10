import selectors
import socket
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Dict

from event_callback.core import BaseComponent, BaseEvent

from .common import MessageEvent


class TCPComponent(BaseComponent):
    # 统一的事件定义
    on_connected = BaseEvent()  # 回调参数: (conn_id, address, is_incoming: bool)
    on_disconnected = BaseEvent()  # 回调参数: (conn_id)
    on_message = MessageEvent()  # 回调参数: (conn_id, data: bytes)

    def __init__(self, router: Callable = None):
        super().__init__()
        self.selector = selectors.DefaultSelector()
        self.connections: Dict[str, socket.socket] = {}
        self.router = router

        self._running = False
        self._net_thread = None

        self.start_engine()

    def start_engine(self):
        """启动底层的 I/O 多路复用引擎"""
        if self._running:
            return
        self._running = True
        self._net_thread = threading.Thread(target=self._network_loop, daemon=True)
        self._net_thread.start()

    # ---- 角色1：作为 Server 提供服务 ----
    def start_server(self, host: str, port: int):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((host, port))
        server_sock.listen(100)
        server_sock.setblocking(False)  # 必须非阻塞

        # 注册到 selector，监听 "可读" 事件（有新连接进来）
        # 附带 data 是为了区分这是 server socket 还是普通通信 socket
        self.selector.register(
            server_sock, selectors.EVENT_READ, data={"type": "server_accept"}
        )
        self.server_socket = server_sock

    # ---- 角色2：作为 Client 主动出击 ----
    def connect_to(self, host: str, port: int) -> str:
        client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_sock.connect((host, port))
        client_sock.setblocking(False)

        conn_id = str(uuid.uuid4())
        self.connections[conn_id] = client_sock

        self.selector.register(
            client_sock,
            selectors.EVENT_READ,
            data={"type": "client_recv", "conn_id": conn_id},
        )

        # 触发连接成功事件 (在线程池执行，不阻塞主线程)
        self.worker_pool.submit(
            self.trigger, "on_connected", {}, conn_id, f"{host}:{port}", False
        )
        return conn_id

    # ---- 统一的发送 API ----
    def send(self, conn_id: str, data: bytes):
        """向指定的连接发送数据"""
        sock = self.connections.get(conn_id)
        if sock:
            try:
                sock.sendall(data)
            except OSError:
                self._handle_disconnect(conn_id)

    def broadcast(self, data: bytes):
        """群发数据给池子里的所有人"""
        for conn_id in list(self.connections.keys()):
            self.publish(conn_id, data)

    # ---- 核心网络引擎 (私有) ----
    def _network_loop(self):
        while self._running:
            # 阻塞等待，直到有 socket 准备好（有新连接、或者有数据来）
            events = self.selector.select(timeout=1.0)
            for key, mask in events:
                sock = key.fileobj
                meta = key.data

                if meta["type"] == "server_accept":
                    self._accept_wrapper(sock)
                elif meta["type"] == "client_recv":
                    self._read_wrapper(sock, meta["conn_id"])

    def _accept_wrapper(self, server_sock):
        conn, addr = server_sock.accept()
        conn.setblocking(False)
        conn_id = str(uuid.uuid4())
        self.connections[conn_id] = conn

        # 将新进来的连接加入监控池
        self.selector.register(
            conn, selectors.EVENT_READ, data={"type": "client_recv", "conn_id": conn_id}
        )

        # 抛入线程池触发事件
        self.trigger("on_connected", {}, conn_id, str(addr), True)

    def _read_wrapper(self, sock, conn_id):
        try:
            data = sock.recv(4096)
            if data:
                # 收到数据，抛入业务线程池处理
                params = {}
                if self.router is not None:
                    params["route"] = self.router(data)
                self.worker_pool.submit(
                    self.trigger, "on_message", params, conn_id, data
                )
            else:
                # 收到了空字节，说明对方正常断开连接
                self._handle_disconnect(conn_id)
        except OSError:
            # 网络异常断开
            self._handle_disconnect(conn_id)

    def _handle_disconnect(self, conn_id):
        if conn_id in self.connections:
            sock = self.connections.pop(conn_id)
            self.selector.unregister(sock)
            sock.close()
            self.trigger("on_disconnected", {}, conn_id)
