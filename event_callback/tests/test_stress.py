# ==========================================
# 测试用例 3: TCP 极高并发压力测试
# ==========================================
import socket
import time
from concurrent.futures import ThreadPoolExecutor

import pytest


@pytest.mark.stress
def test_99_stress_tcp_concurrent(test_env):
    """
    压测目标: 500 个并发客户端，各发送 10 条消息
    验证点: 框架底层 selector 是否能抗住高压，业务线程池是否丢包
    """
    tcp_port = test_env["tcp_port"]
    manager = test_env["manager"]

    CLIENT_COUNT = 50
    MSG_PER_CLIENT = 10

    def single_client_task(client_id):
        """单个虚拟客户端的行为模型"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("127.0.0.1", tcp_port))
            sock.recv(1024)  # 等待 READY
            sock.settimeout(3.0)

            for _ in range(MSG_PER_CLIENT):
                sock.sendall(b"ping")
                resp = sock.recv(1024)
                if resp != b"pong":
                    return False  # 收到错误回复

            sock.close()
            return True
        except Exception:
            return False

    # 记录压测开始时间
    start_time = time.time()
    # 使用多线程池，瞬间爆发 500 个客户端并发攻击服务器
    with ThreadPoolExecutor(max_workers=CLIENT_COUNT) as pool:
        # 提交所有任务
        futures = [pool.submit(single_client_task, i) for i in range(CLIENT_COUNT)]

        # 收集所有客户端的执行结果
        results = [f.result(timeout=1) for f in futures]
    # 留出一点缓冲时间让服务器消化最后的请求
    time.sleep(0.5)

    duration = time.time() - start_time
    # --- 工业级断言 ---
    # 1. 确保所有虚拟客户端都执行成功，没有报错退出的
    assert (
        all(results) is True
    ), f"存在压测客户端请求失败/超时: [{sum(results)}/{len(results)}]"

    # 2. 核心验证：服务端 Manager 是否一字不差地收到了所有连接和数据！
    assert (
        manager.stats["tcp_connected"] == CLIENT_COUNT
    ), f"服务端丢失了部分连接: [{manager.stats['tcp_connected']}/{CLIENT_COUNT}]"

    expected_msg_count = CLIENT_COUNT * MSG_PER_CLIENT
    assert (
        manager.stats["tcp_msg_count"] == expected_msg_count
    ), f"发生丢包！预期: {expected_msg_count}, 实际: {manager.stats['tcp_msg_count']}"
    # 作为性能参考指标打印（在 pytest -s 模式下可见）
    qps = expected_msg_count / duration
    record = test_env["record"]
    record.append(
        f"\n[压测报告] 耗时: {duration:.2f}秒 | 吞吐量(QPS): {qps:.0f} msg/sec | 无任何死锁/丢包"
    )
