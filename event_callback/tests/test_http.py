# ==========================================
# 测试用例 2: HTTP 路由验证
# ==========================================
import asyncio
import concurrent
import concurrent.futures
import json
from logging import getLogger
from queue import Queue
import threading
import time
import urllib
from threading import Event, Thread
from typing import Any, Callable, Dict
import websockets

import pytest

from tests.conftest import TestManager

def wait_proxy_ready(manager: TestManager):
    for i in range(100):
        if manager.stats["ready"]:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("等待proxy超时！")

def get_url(raw_url: str, test_proxy: bool):
    if test_proxy:
        raw_url = f"/proxy{raw_url}"
    return raw_url


def send_request(url, port, event: Event = None, is_success: Callable = None):
    req_url = f"http://localhost:{port}{url}"
    response = urllib.request.urlopen(req_url)

    assert response.status == 200
    raw_text = response.read().decode()
    actual_data = json.loads(raw_text)
    if is_success is None:
        assert actual_data == "SUCCESS"
    else:
        is_success(actual_data)
    if event is not None:
        event.set()


def _test_http_01(test_env: Dict[str, Any], test_proxy: bool):
    """验证 HTTP 组件能正确解析并路由请求"""
    http_port = test_env["http_port"]
    manager = test_env["manager"]
    
    if test_proxy:
        wait_proxy_ready(manager)
        
    # time.sleep(1)
    event = Event()
    with concurrent.futures.ThreadPoolExecutor() as excutor:
        res = excutor.submit(
            send_request, get_url("/test1", test_proxy), http_port, event
        )

    event.wait(0.1)
    res = res.result()
    assert manager.stats["http_req_count"] == 1


@pytest.mark.http
def test_http_01(test_env):
    _test_http_01(test_env, False)


@pytest.mark.http
def test_http_proxy_01(test_env):
    _test_http_01(test_env, True)
    # time.sleep(10000)


def _test_http_02(test_env: Dict[str, Any], test_proxy: bool):
    """验证一个请求阻塞的时候是否影响其他请求处理"""
    http_port = test_env["http_port"]
    manager = test_env["manager"]
    # time.sleep(1)
    if test_proxy:
        wait_proxy_ready(manager)
        
    event = threading.Event()
    with concurrent.futures.ThreadPoolExecutor() as excutor:
        f1 = excutor.submit(send_request, get_url("/forever", test_proxy), http_port)
        f2 = excutor.submit(
            send_request, get_url("/test1", test_proxy), http_port, event
        )

    try:
        event.wait(1)
    except Exception as e:
        assert False, "请求被阻塞!"
    a = f1.result()
    b = f2.result()
    time.sleep(0.1)
    assert manager.stats["http_req_count"] == 1


@pytest.mark.http
def test_http_02(test_env):
    _test_http_02(test_env, False)


@pytest.mark.http
def test_http_proxy_02(test_env):
    _test_http_02(test_env, True)


@pytest.mark.http
def test_http_proxy_record(test_env):
    """测试使用proxy如果调用被阻塞，是否会影响其他调用"""
    http_port = test_env["http_port"]
    manager = test_env["manager"]
    # time.sleep(1)
    wait_proxy_ready(manager)
    
    event = threading.Event()
    with concurrent.futures.ThreadPoolExecutor() as excutor:
        f1 = excutor.submit(
            send_request,
            "/stop_record",
            http_port,
            is_success=lambda x: x["status"] == "error"
            and x["msg"]
            == "timeout exceeded while waiting for service /no_such_service",
        )
        for i in range(5):
            f2 = excutor.submit(send_request, get_url("/test1", True), http_port, event)

    try:
        event.wait(1)
    except Exception as e:
        assert False, "请求被阻塞!"
    a = f1.result()
    b = f2.result()
    time.sleep(0.1)
    assert manager.stats["http_req_count"] == 5

logger = getLogger(__name__)

@pytest.mark.ws
@pytest.mark.asyncio
async def test_ws_ack(test_env):
    """测试websocket上报重试逻辑"""
    HOST = "localhost"
    PORT = test_env["http_port"]
    uri = f"ws://{HOST}:{PORT}/ws"

    # 直接在同一个事件循环中建立连接
    async with websockets.connect(uri) as websocket:
        # 发送一个事件
        manager = test_env["manager"]
        manager.http_node.publish({"type": "event", "event": "test_event"})
        
        # 1. 接收第一条数据 (带超时机制，代替 Queue.get)
        try:
            msg = await asyncio.wait_for(websocket.recv(), timeout=5.0)
        except asyncio.TimeoutError:
            pytest.fail("未在5秒内收到初始事件数据！")
            
        data = json.loads(msg)
        assert data.get("event") == "test_event", "未收到正确的事件数据！"
        msg_id = data.get("msg_id")
        assert msg_id is not None, "未收到msg_id！"
        # 2. 等待重试消息 (假设服务器3秒内会重试，这里给5秒超时)
        try:
            retry_msg = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            retry_data = json.loads(retry_msg)
            assert retry_data.get("msg_id") == msg_id, f"重试消息id错误！: {retry_data.get('msg_id')}"
        except asyncio.TimeoutError:
            pytest.fail("未收到服务器的重试消息！")
        # 3. 发送 ACK 回执 (这里使用了 await，真正发送出去了！)
        await websocket.send(json.dumps({"msg_id": msg_id}))
        # 4. 验证发送 ACK 后不再收到重试消息
        try:
            # 故意等待一段时间，看是否还有多余的数据发过来
            extra_msg = await asyncio.wait_for(websocket.recv(), timeout=3.0)
            pytest.fail(f"发送ACK后仍收到了多余的重试消息: {extra_msg}")
        except asyncio.TimeoutError:
            # 触发超时是符合预期的，说明没有多余的消息了
            pass
