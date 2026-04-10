# ==========================================
# 测试用例 1: TCP 全生命周期验证
# ==========================================
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import pytest


@pytest.mark.udp
def test_01_udp_lifecycle(test_env):
    """验证客户端连接、收发消息、断开的完整闭环"""
    udp_port = test_env["udp_port"]
    manager = test_env["manager"]

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.settimeout(2.0)

    udp_socket.bind(("0.0.0.0", udp_port + 1))

    def recv_worker():
        data, client_addr = udp_socket.recvfrom(1024)
        assert data == b"pong", "业务路由未正确返回 pong"

    executor = ThreadPoolExecutor(max_workers=1)
    future = executor.submit(recv_worker)
    udp_socket.sendto(b"ping", ("127.0.0.1", udp_port))
    result = future.result()
    udp_socket.close()

    # 动作 3：验证 Manager 状态
    assert manager.stats["udp_msg_count"] == 1
