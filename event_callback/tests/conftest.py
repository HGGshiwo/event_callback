import threading
import time

import pytest
import rospy
import std_msgs
from mavproxy_ros.utils import wait_for_debugger

from event_callback.components.http.core import HTTPComponent
from event_callback.components.ros import ROSComponent
from event_callback.components.socket.tcp import TCPComponent
from event_callback.components.socket.udp import UDPComponent
from event_callback.core import BaseManager
from event_callback.utils import setup_logger

# 假设导入你的框架基类
# from framework import BaseManager, TcpServerComponent, HttpComponent
setup_logger()


# --- 1. 专门用于测试的业务 Manager ---
class TestManager(BaseManager):
    def __init__(
        self,
        tcp_node: TCPComponent,
        http_node: HTTPComponent,
        ros_node: ROSComponent,
        udp_node: UDPComponent,
    ):
        super().__init__(tcp_node, http_node, ros_node, udp_node)
        tcp_node.start_server(host="127.0.0.1", port=0)
        udp_node.start_server(host="127.0.0.1", port=0)
        http_node.start_server(host="0.0.0.0", port=0)
        self.tcp_node = tcp_node
        self.udp_node = udp_node
        self.ros_node = ros_node

        # 线程安全的计数器（压测必备）
        self.lock = threading.Lock()
        self.stats = {
            "tcp_connected": 0,
            "tcp_msg_count": 0,
            "udp_msg_count": 0,
            "http_req_count": 0,
            "ros_msg_count": 0,
        }

    @UDPComponent.on_message("/test_udp")
    def on_udp_msg(self, data, addr):
        if data == b"ping":
            with self.lock:
                self.stats["udp_msg_count"] += 1
            udp_port = self.udp_node._socket.getsockname()[1]
            self.udp_node.send(b"pong", "127.0.0.1", udp_port + 1)

    @TCPComponent.on_connected()
    def on_tcp_connect(self, conn_id, address, is_incoming):
        with self.lock:
            self.stats["tcp_connected"] += 1
        self.tcp_node.send(conn_id, b"READY")

    @TCPComponent.on_message("/test_tcp")
    def on_tcp_msg1(self, conn_id, data):
        with self.lock:
            self.stats["tcp_msg_count"] += 1
        if data == b"ping":
            self.tcp_node.send(conn_id, b"pong")

    @HTTPComponent.on_get("/test1")
    def on_http(self):
        with self.lock:
            self.stats["http_req_count"] += 1
        return "SUCCESS"

    @ROSComponent.on_topic("/test", std_msgs.msg.String, 1)
    def on_topic(self, data):
        with self.lock:
            if data.data == "Hello":
                self.stats["ros_msg_count"] += 1


# --- 2. 工业级环境夹具 (Fixtures) ---
# scope="function" 意味着每个 test_xxx 函数运行前都会初始化一套全新的环境
@pytest.fixture(scope="function")
def test_env(request):
    """自动化生命周期管理，提供隔离的测试环境"""
    # 强制端口为0，让操作系统分配，杜绝端口冲突
    tcp_comp = TCPComponent(router=lambda x: "/test_tcp")
    http_comp = HTTPComponent()
    udp_comp = UDPComponent(router=lambda x: "/test_udp")
    ros_comp = ROSComponent()
    manager = TestManager(tcp_comp, http_comp, ros_comp, udp_comp)

    # 通过 yield 把环境交给具体的测试用例使用
    record = []

    for i in range(100):
        if hasattr(http_comp.server, "servers"):
            break
        time.sleep(0.1)
    else:
        assert 0 == 1, "timeout!"
    yield {
        "manager": manager,
        "tcp_port": tcp_comp.server_socket.getsockname()[1],
        "udp_port": udp_comp._socket.getsockname()[1],
        "http_port": http_comp.server.servers[0].sockets[0].getsockname()[1],
        "ros_topic": "/test",
        "record": record,
    }
    if not hasattr(request.session, "record"):
        request.session.record = []
    request.session.record += record


@pytest.fixture(scope="session", autouse=True)
def global_setup():
    """所有测试开始前执行一次，结束后执行一次"""
    # wait_for_debugger()
    from rosmaster.master import Master

    master = Master()
    master.start()
    rospy.init_node("test")
    print("start master")

    yield  # 此处分隔启动和清理代码
    master.stop()


def pytest_sessionfinish(session, exitstatus):
    if hasattr(session, "record"):
        for r in session.record:
            print(r)


def pytest_configure(config):
    config.addinivalue_line("markers", "http")
    config.addinivalue_line("markers", "ros")
    config.addinivalue_line("markers", "stress")
    config.addinivalue_line("markers", "tcp")
    config.addinivalue_line("markers", "udp")
