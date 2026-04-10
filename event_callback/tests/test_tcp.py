# ==========================================
# 测试用例 1: TCP 全生命周期验证
# ==========================================
import socket
import time

import pytest


@pytest.mark.tcp
def test_01_tcp_lifecycle(test_env):
    """验证客户端连接、收发消息、断开的完整闭环"""
    tcp_port = test_env["tcp_port"]
    manager = test_env["manager"]

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(2.0)

    # 动作 1：连接并接收欢迎语
    client.connect(("127.0.0.1", tcp_port))
    assert client.recv(1024) == b"READY", "未收到服务端的就绪信号"

    # 动作 2：发送消息并接收回复
    client.sendall(b"ping")
    assert client.recv(1024) == b"pong", "业务路由未正确返回 pong"

    client.close()
    time.sleep(0.1)  # 稍等异步事件处理

    # 动作 3：验证 Manager 状态
    assert manager.stats["tcp_connected"] == 1
    assert manager.stats["tcp_msg_count"] == 1
