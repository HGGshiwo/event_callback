from dataclasses import dataclass, field
import errno
import socket
import asyncio
import threading
from functools import partial
import logging
from typing import (
    Any,
    Awaitable,
    Dict,
    List,
    Callable,
    Optional,
    Tuple,
    Type,
    Union,
)

try:
    import rospy
    from std_msgs.msg import Empty
    from event_callback_msg.srv import RegisterRequest
except:
    rospy = None

    class RegisterRequest:
        pass


from event_callback.utils import get_classname, rospy_is_shutdown

# 组件基类与类型导入
from event_callback.core import (
    R,
    BaseComponent,
    BaseComponentHelper,
    BaseConfig,
    CallbackItem,
    CallbackManager,
)

# 公共类型别名
ConnType = Union[socket.socket, None]
AddrType = Union[Tuple[str, int], None]
DataHandleFunc = Callable[[bytes, AddrType], Awaitable[None]]

logger = logging.getLogger(__name__)

class BaseSocketManager:
    """公共Socket连接管理基类：抽离服务端/客户端通用的连接、数据处理逻辑"""

    def __init__(self, component):
        self.component = component  # 关联父组件（BaseSocketComponent）
        self.lock = threading.Lock()  # 全局线程安全锁

    async def safe_send(self, conn: socket.socket, data: bytes, addr: AddrType = None):
        """通用安全发送方法：适配TCP/UDP+Python3.6+，自动捕获发送异常
        :param conn: 套接字对象
        :param data: 待发送字节数据
        :param addr: UDP目标地址(TCP忽略)，格式(ip, port)
        """
        try:
            loop = asyncio.get_event_loop()
            if self.component.socket_type == socket.SOCK_DGRAM and addr:
                # 低版本兼容：UDP用run_in_executor执行sendto，替代3.9+的sock_sendto
                await loop.run_in_executor(None, conn.sendto, data, addr)
            else:
                # TCP：3.6+原生支持sock_sendall，无需兼容
                await loop.sock_sendall(conn, data)
        except Exception as e:
            logger.exception(f"Socket send error: {str(e)}")

    async def decode_data(self, data: bytes, decode: bool = False):
        """通用数据解码：统一异常处理，返回解码后的字符串/原字节"""
        try:
            if not decode:
                return data
            data_str = data.decode("utf8").strip()
            return data_str if data_str else None
        except Exception as e:
            logger.exception(f"Socket decode error: {str(e)}")
            return None

    async def route_callback(self, data_str: Union[str, bytes], addr: AddrType = None):
        """通用回调路由核心：提取ID→映射URL→执行回调（服务端/客户端/TCP/UDP完全复用）"""
        if not self.component.id_extractor:
            logger.error(f"{get_classname(self, False)} callback failed: no id_exctractor found")
            return
        if not self.component.callback_map:
            logger.error(f"{get_classname(self, False)}callback failed: no callback found")
            return
        # 提取业务ID并映射回调URL
        try:
            callback_url = self.component.id_extractor(data_str)
            if callback_url not in self.component.callback_map:
                logger.error(f"No callback match ID: {callback_url}!")
                return
        except Exception as e:
            logger.error(f"Socket extract id failed: {str(e)}")
            return

        # 执行回调（兼容同步/异步，入参统一）
        try:
            callback = self.component.callback_map[callback_url]
            if asyncio.iscoroutinefunction(callback):
                await callback(data_str)
            else:
                callback(data_str)
        except Exception as e:
            logger.exception(f"Socket excute callback failed: {str(e)}")


class SocketServerManager(BaseSocketManager):
    """Socket服务端专属管理器：继承公共基类，扩展多客户端连接管理（仅TCP生效）"""

    def __init__(self, component):
        super().__init__(component)
        self.connections: List[Tuple[socket.socket, AddrType]] = (
            []
        )  # TCP客户端连接列表（UDP忽略）

    def add_conn(self, conn: socket.socket, addr: AddrType):
        """服务端：添加客户端连接（线程安全，仅TCP生效）"""
        if self.component.socket_type == socket.SOCK_DGRAM:
            return
        with self.lock:
            self.connections.append((conn, addr))
        logger.info(f"TCP server: New client {addr}, current connections: {len(self.connections)}")

    def remove_conn(self, conn: socket.socket, addr: AddrType):
        """服务端：移除客户端连接（线程安全，自动关闭socket，仅TCP生效）"""
        if self.component.socket_type == socket.SOCK_DGRAM:
            return
        with self.lock:
            if (conn, addr) in self.connections:
                self.connections.remove((conn, addr))
        try:
            conn.close()
        except Exception:
            pass
        logger.info(f"TCP Server: client disconnected {addr}, current connections: {len(self.connections)}")


class SocketClientManager(BaseSocketManager):
    """Socket客户端专属管理器：继承公共基类，扩展单连接+断连自动重连（TCP真连接/UDP伪连接）"""

    def __init__(self, component):
        super().__init__(component)
        self.conn: ConnType = None  # 客户端套接字
        self.addr: AddrType = None  # 服务端地址
        self.is_connected = False  # 连接状态（TCP：真连接/UDP：伪连接）
        self.reconnect_task: Optional[asyncio.Task] = None  # 重连异步任务

    async def connect(self, host: str, port: int, socket_type: Any) -> bool:
        """客户端：主动连接服务端（TCP真连接/UDP伪连接），返回连接结果"""
        if self.is_connected:
            logger.warning(f"{socket_type.name} Client: already connected!")
            return True
        try:
            self.conn = socket.socket(socket.AF_INET, socket_type)
            self.conn.setblocking(False)
            self.addr = (host, port)
            loop = asyncio.get_event_loop()

            if socket_type == socket.SOCK_STREAM:
                # TCP：三次握手建立真连接
                await loop.sock_connect(self.conn, self.addr)
            else:
                # UDP：伪连接（仅固定目标地址，无实际连接过程）
                self.conn.connect(self.addr)

            self.is_connected = True
            logger.info(f"{socket_type.name} Client: connected to {host}:{port}")
            return True
        except Exception as e:
            logger.exception(
                f"{socket_type.name} Client: connected to {host}:{port} failed: {str(e)}"
            )
            self.conn = None
            self.addr = None
            self.is_connected = False
            return False

    def disconnect(self):
        """客户端：主动断开连接（线程安全，TCP关闭连接/UDP重置状态）"""
        with self.lock:
            if self.is_connected and self.conn:
                try:
                    self.conn.close()
                except Exception:
                    pass
                self.is_connected = False
                self.conn = None
                logger.info(f"Socket Client: disconnected {self.addr}")

    async def auto_reconnect(self):
        """客户端：断连自动重连（后台异步运行，TCP/UDP通用）"""
        if self.reconnect_task and not self.reconnect_task.done():
            return
        reconnect_count = 0
        self.reconnect_task = asyncio.current_task()
        while rospy_is_shutdown() and not self.is_connected:
            # 达到最大重连次数则停止（-1表示无限重连）
            if (
                self.component.max_reconnect != -1
                and reconnect_count >= self.component.max_reconnect
            ):
                logger.info(
                    f"Socket Client: Reached maximum reconnection attempts {self.component.max_reconnect}, stop reconnecting"
                )
                break
            # 尝试重连（传socket_type修复原代码缺参bug）
            logger.info(f"Socket Client: Attempting to reconnect to server for the {reconnect_count+1}th time...")
            if await self.connect(
                self.component.socket_host,
                self.component.socket_port,
                self.component.socket_type,
            ):
                break
            reconnect_count += 1
            await asyncio.sleep(self.component.reconnect_interval)


class BaseSocketComponent(BaseComponent):
    """Socket公共组件基类：抽离服务端/客户端通用的配置、ROS、回调逻辑
    服务端/客户端/TCP/UDP均继承此类，实现最大程度复用
    """

    def __init__(self, is_server: bool, **config: Dict[str, Any]):
        super().__init__(**config)
        self.is_server = is_server  # 标识服务端/客户端
        self._parse_common_config()  # 解析通用配置
        self._init_core_var()  # 初始化核心变量
        if self.enable_register:
            self._init_ros_node()  # 初始化ROS节点
            self._init_ros_pub_sub()  # 初始化ROS Pub/Sub
        self._init_socket_manager()  # 初始化专属Socket管理器
        self.do_register_trigger()  # 触发ROS节点注册Socket服务

    def _is_nonblocking_normal_error(self, e: Exception) -> bool:
        """判断是否为非阻塞IO的正常错误（errno 11/资源暂时不可用）"""
        if not isinstance(e, OSError):
            return False
        # 匹配errno 11（Resource temporarily unavailable）
        return e.errno == errno.EAGAIN or e.errno == errno.EWOULDBLOCK
        # 注：EAGAIN和EWOULDBLOCK在Linux下是同一个值（11），兼容所有系统

    def _get_socket_type(self):
        """通过配置type获取套接字类型，统一小写判断"""
        _type = self.config.get("type", "tcp").lower()
        if _type == "udp":
            return socket.SOCK_DGRAM
        elif _type == "tcp":
            return socket.SOCK_STREAM
        raise KeyError(f"Unsupported Socket type: {_type}, only tcp/udp are supported")

    def _parse_common_config(self):
        """解析服务端/客户端/TCP/UDP通用配置"""
        # 基础网络配置
        host = "0.0.0.0" if self.is_server else "127.0.0.1"
        self.socket_host = self.config.get("host", host)
        self.socket_port = self.config.get("port", 12345)
        self.socket_type = self._get_socket_type()  # TCP/UDP类型标识
        self.id_extractor: Optional[Callable] = self.config.get("router")  # ID提取函数
        self.buffer_size = self.config.get("buffer_size", 256)  # 接收缓冲区大小
        self.decode = self.config.get("decode", False)  # 是否解码为字符串
        # ROS注册配置
        self.enable_register = self.config.get("register", True)
        self.ros_topic_prefix = self.config.get("ros_topic_prefix", "socket")
        # 客户端专属配置（服务端忽略）
        # 重连间隔(秒)
        self.reconnect_interval = self.config.get("reconnect_interval", 3.0)
        # 最大重连次数(-1=无限)
        self.max_reconnect = self.config.get("max_reconnect", -1)
        # 异步/线程配置
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.socket_thread: Optional[threading.Thread] = None

    def _init_core_var(self):
        """初始化通用核心变量（回调路由相关）"""
        self.callback_map = {}  # 回调函数映射: {url: callback_func}
        self.callback_lock = threading.Lock()  # 回调注册线程锁

    def _init_ros_node(self):
        """初始化ROS节点（服务端/客户端节点名区分，避免冲突）"""
        node_suffix = "server" if self.is_server else "client"
        node_name = self.config.get(
            "node_name",
            f"{self.__class__.__name__.lower()}_{self.ros_topic_prefix}_{node_suffix}",
        )
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=False)
            logger.info(f"Socket{node_suffix.capitalize()} initialized ROS node: {node_name}")

    def _init_ros_pub_sub(self):
        """初始化ROS Pub/Sub（服务端/客户端话题名区分，避免冲突）"""
        register_topic = f"{self.ros_topic_prefix}_{'server' if self.is_server else 'client'}/register"
        do_register_topic = f"{self.ros_topic_prefix}_{'server' if self.is_server else 'client'}/do_register"

        self.do_register_pub = rospy.Publisher(
            do_register_topic, Empty, queue_size=10, latch=True
        )
        self.register_sub = rospy.Subscriber(
            register_topic, RegisterRequest, self._ros_register_callback, queue_size=10
        )
        logger.info(
            f"Socket{'Server' if self.is_server else 'Client'} ROS Pub/Sub initialized: "
            f"Pub={do_register_topic} | Sub={register_topic}"
        )

    def _init_socket_manager(self):
        """初始化专属Socket管理器（服务端/客户端分别实例化）"""
        self.socket_mgr = (
            SocketServerManager(self) if self.is_server else SocketClientManager(self)
        )

    def _ros_register_callback(self, req: RegisterRequest):
        """通用ROS注册回调（服务端/客户端通用，处理ROS端动态注册请求）"""
        try:
            callback_url = req.path
            if not callback_url:
                logger.error("Socket ROS registration failed: path parameter is empty")
                return
            logger.info(
                f"Socket{'Server' if self.is_server else 'Client'} dynamically registered callback from ROS: {callback_url}"
            )
        except Exception as e:
            logger.error(f"Socket ROS registration callback failed: {str(e)}")

    def do_register_trigger(self):
        """通用注册触发方法：向ROS发布Empty消息，触发其他节点注册"""
        if not self.enable_register:
            logger.warning("Socket component has disabled ROS registration, cannot trigger do_register")
            return
        self.do_register_pub.publish(Empty())
        logger.info(
            f"Socket{'Server' if self.is_server else 'Client'} triggered ROS do_register message publication"
        )

    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        """实现BaseComponent抽象方法：通用回调注册入口（全场景复用）
        @param callbacks: 回调列表，CallbackItem的args格式：(url)
                          - url: 回调唯一标识
        """
        for callback, args, kwargs in callbacks:
            callback_url = args[0]
            # 初始化全局唯一的ID提取函数和映射表（仅首次注册生效）
            # 线程安全注册回调
            with self.callback_lock:
                self.callback_map[callback_url] = partial(callback, **kwargs)
                logger.info(
                    f"Socket{'Server' if self.is_server else 'Client'} callback registered successfully: "
                    f"{callback_url.name} -> {callback.__name__}"
                )

    def _start_async_thread(self, coro):
        """通用异步线程启动方法：与ROS主线程解耦，全场景复用"""
        if self.socket_thread and self.socket_thread.is_alive():
            logger.info(
                f"Socket{'Server' if self.is_server else 'Client'} is already running, no need to start again"
            )
            return
        # 守护线程：ROS退出时自动终止
        self.socket_thread = threading.Thread(
            target=asyncio.run, args=(coro,), daemon=True
        )
        self.socket_thread.start()
        logger.info(f"Socket{'Server' if self.is_server else 'Client'} async thread started")


class SocketServerComponent(BaseSocketComponent):
    """Socket服务端组件：支持TCP/UDP，仅实现服务端特有网络逻辑
    配置通过type指定协议：type: tcp/udp
    :param str host: 监听IP
    :param int port: 监听端口
    :param Callable router: 数据包ID提取函数
    :param int buffer_size: 接收缓冲区大小
    """

    def __init__(self, **config: Dict[str, Any]):
        super().__init__(is_server=True, **config)
        self.server_sock: ConnType = None
        self.start_socket_server()  # 启动服务端

    async def _handle_tcp_client(self, conn: socket.socket, addr: AddrType):
        """TCP服务端专属：异步处理单个客户端连接，循环接收数据"""
        self.socket_mgr.add_conn(conn, addr)
        # 发送连接成功提示
        await self.socket_mgr.safe_send(conn, b"connected to tcp socket server")
        try:
            while not rospy_is_shutdown():
                try:
                    # TCP基于连接接收数据，空数据表示客户端断开
                    data = await self.loop.sock_recv(conn, self.buffer_size)
                    if not data:
                        break
                    # 解码并路由回调
                    if data_str := await self.socket_mgr.decode_data(
                        data, decode=self.decode
                    ):
                        await self.socket_mgr.route_callback(data_str, addr)
                except Exception as e:
                    if self._is_nonblocking_normal_error(e):
                        await asyncio.sleep(0.05)
                        continue
                    raise e
        except Exception as e:
            if not rospy_is_shutdown():
                logger.error(f"TCP server error handling client {addr}: {str(e)}")
        finally:
            self.socket_mgr.remove_conn(conn, addr)

    async def _run_udp_server(self):
        """UDP服务端专属：异步主逻辑，无连接，循环接收数据报（兼容Python3.6+）"""
        while not rospy_is_shutdown():
            try:
                # 低版本兼容：用run_in_executor执行recvfrom，替代3.9+的sock_recvfrom
                data, addr = await self.loop.run_in_executor(
                    None, self.server_sock.recvfrom, self.buffer_size
                )
                if not data:
                    continue
                # 解码并路由回调（携带发送方地址）
                if data_str := await self.socket_mgr.decode_data(
                    data, decode=self.decode
                ):
                    await self.socket_mgr.route_callback(data_str, addr)
            except Exception as e:
                if rospy_is_shutdown():
                    break
                if self._is_nonblocking_normal_error(e):
                    await asyncio.sleep(0.05)
                    continue
                logger.error(f"UDP server error receiving data: {str(e)}")
                await asyncio.sleep(0.1)

    async def _run_tcp_server(self):
        """TCP服务端专属：异步主逻辑，监听+接受连接+多客户端处理"""
        self.server_sock.listen(8)  # 开启TCP监听
        logger.info(f"TCP server started listening on: {self.socket_host}:{self.socket_port}")
        while not rospy_is_shutdown():
            try:
                # 接受TCP客户端连接，返回新的通信套接字
                conn, addr = await self.loop.sock_accept(self.server_sock)
                # 为每个客户端创建独立异步任务，实现并发处理
                self.loop.create_task(self._handle_tcp_client(conn, addr))
            except Exception as e:
                if rospy_is_shutdown():
                    break
                if self._is_nonblocking_normal_error(e):
                    await asyncio.sleep(0.05)  # 短暂延时，减少CPU空转
                    continue
                # 其他错误才打印日志
                logger.error(f"TCP server error accepting connection: {str(e)}")
                await asyncio.sleep(0.1)
        # 服务端关闭，清理所有TCP客户端连接
        for conn, addr in self.socket_mgr.connections:
            self.socket_mgr.remove_conn(conn, addr)

    async def _run_server(self):
        """服务端统一入口：按socket_type分支执行TCP/UDP逻辑"""
        # 初始化通用服务端套接字（TCP/UDP共用）
        self.server_sock = socket.socket(socket.AF_INET, self.socket_type)
        self.server_sock.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )  # 端口复用
        self.server_sock.bind((self.socket_host, self.socket_port))
        self.server_sock.setblocking(False)
        self.loop = asyncio.get_event_loop()

        # 分支执行TCP/UDP逻辑
        if self.socket_type == socket.SOCK_STREAM:
            await self._run_tcp_server()
        else:
            logger.info(f"UDP server started listening on: {self.socket_host}:{self.socket_port}")
            await self._run_udp_server()

        # 服务端关闭，释放套接字
        self.server_sock.close()

    def start_socket_server(self):
        """服务端对外暴露的启动方法：启动异步线程"""
        self._start_async_thread(self._run_server())


class SocketClientComponent(BaseSocketComponent):
    """Socket客户端组件：支持TCP/UDP，仅实现客户端特有网络逻辑

    :param tcp/udp type: 配置通过type指定协议
    :param str host: 服务端IP
    :param int port: 服务端端口
    :param Callable router: 数据包 -> callback_url
    :param int buffer_size: 接收缓冲区大小
    :param float reconnect_interval: 重连间隔（秒）
    :param int max_reconnect: 最大重连次数（-1=无限）
    """

    def __init__(self, **config: Dict[str, Any]):
        super().__init__(is_server=False, **config)
        self.start_socket_client()  # 启动客户端

    async def _recv_data_loop(self):
        """客户端通用接收循环：按socket_type分支处理TCP/UDP（兼容Python3.6+）"""
        while not rospy_is_shutdown() and self.socket_mgr.is_connected:
            try:
                if not self.socket_mgr.conn:
                    break
                # 分支接收TCP/UDP数据，UDP做低版本兼容
                try:
                    if self.socket_type == socket.SOCK_STREAM:
                        data = await self.loop.sock_recv(
                            self.socket_mgr.conn, self.buffer_size
                        )
                        addr = self.socket_mgr.addr
                    else:
                        # 低版本兼容：UDP用run_in_executor执行recvfrom，替代3.9+的sock_recvfrom
                        data, addr = await self.loop.run_in_executor(
                            None, self.socket_mgr.conn.recvfrom, self.buffer_size
                        )
                except Exception as e:
                    if self._is_nonblocking_normal_error(e):
                        await asyncio.sleep(0.05)
                        continue
                    raise e  # 其他错误抛出

                if not data:  # 空数据表示TCP服务端断开，UDP无此情况
                    logger.info("TCP Client: Server disconnected actively")
                    self.socket_mgr.disconnect()
                    self.loop.create_task(self.socket_mgr.auto_reconnect())
                    break
                # 解码并路由回调
                if data_str := await self.socket_mgr.decode_data(
                    data, decode=self.decode
                ):
                    await self.socket_mgr.route_callback(data_str, addr)
            except Exception as e:
                if not rospy_is_shutdown() and self.socket_mgr.is_connected:
                    logger.error(f"Socket Client error receiving data: {str(e)}")
                    self.socket_mgr.disconnect()
                    self.loop.create_task(self.socket_mgr.auto_reconnect())
                break

    async def _run_client(self):
        """客户端统一入口：TCP/UDP通用连接+接收+重连逻辑"""
        self.loop = asyncio.get_event_loop()
        # 首次连接服务端
        if not await self.socket_mgr.connect(
            self.socket_host, self.socket_port, self.socket_type
        ):
            await self.socket_mgr.auto_reconnect()
        # 持续接收数据，断开则自动重连
        while not rospy_is_shutdown():
            if self.socket_mgr.is_connected and self.socket_mgr.conn:
                await self._recv_data_loop()
            else:
                await asyncio.sleep(0.5)
        # 客户端关闭，主动断开
        self.socket_mgr.disconnect()

    def start_socket_client(self):
        """客户端对外暴露的启动方法：启动异步线程"""
        self._start_async_thread(self._run_client())

    def send_to_server(self, data: Union[str, bytes]):
        """客户端对外暴露的发送方法：TCP/UDP通用，线程安全
        :param data: 待发送数据（字符串/字节流）
        """
        if not self.socket_mgr.is_connected or not self.socket_mgr.conn:
            logger.error("Socket Client: Not connected to server, failed to send data")
            return
        # 统一转换为字节流
        data_bytes = data.encode("utf8") if isinstance(data, str) else data
        # 异步发送（TCP传None，UDP传服务端地址）
        asyncio.run_coroutine_threadsafe(
            self.socket_mgr.safe_send(
                self.socket_mgr.conn, data_bytes, self.socket_mgr.addr
            ),
            self.loop,
        )
        # 日志打印（忽略非UTF8数据解码错误）
        logger.info(f"Socket Client sent data to server: 0x{data_bytes.hex()}")

@dataclass
class SocketConfig(BaseConfig):
    host: str # 监听/连接的IP
    port: int # 监听/连接的端口
    socket_type: str # udp or tcp
    router: Optional[Callable] = None # 数据包 -> callback_url
    decode: Optional[bool] = False # 是否需要解码为字符串
    buffer_size: Optional[int] = 256 # 接收缓冲区大小
    register: Optional[bool] = False # 是否提供ros注册接口

@dataclass
class SocketClientConfig(SocketConfig):
    target: type = field(default=SocketClientComponent, init=False)

@dataclass
class SocketClientConfig(SocketConfig):
    target: type = field(default=SocketClientComponent, init=False)


class sockets(BaseComponentHelper):
    target = SocketServerComponent

    @classmethod
    def recv(cls, url: str, frequency: Optional[int] = None):
        return R._create_comp_decorator(SocketServerComponent, url, frequency=frequency)

class socketc(BaseComponentHelper):
    target: Type["SocketClientComponent"] = SocketClientComponent

    @classmethod
    def send_to_server(cls, manager: CallbackManager, data: Dict[str, Any]):
        return manager.get_component_instance(cls.target).send_to_server(data)

    @classmethod
    def is_connected(cls, manager: CallbackManager):
        """判断socket clinet连接状态"""
        comp = manager.get_component_instance(cls.target)
        return comp.socket_mgr.is_connected and comp.socket_mgr.conn