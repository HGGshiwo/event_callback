# ==========================================
# 测试用例 2: HTTP 路由验证
# ==========================================
import concurrent
import concurrent.futures
import json
import threading
import time
import urllib
from threading import Event, Thread
from typing import Any, Callable, Dict

import pytest


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
    http_port = test_env["http_port"]
    manager = test_env["manager"]
    # time.sleep(1)

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
