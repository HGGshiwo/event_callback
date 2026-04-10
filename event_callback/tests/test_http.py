# ==========================================
# 测试用例 2: HTTP 路由验证
# ==========================================
import json
import time
import urllib

import pytest


@pytest.mark.http
def test_02_http_routing(test_env):
    """验证 HTTP 组件能正确解析并路由请求"""
    http_port = test_env["http_port"]
    manager = test_env["manager"]
    # time.sleep(1)
    req_url = f"http://localhost:{http_port}/test1"
    response = urllib.request.urlopen(req_url)

    assert response.status == 200
    raw_text = response.read().decode()
    actual_data = json.loads(raw_text)
    assert actual_data == "SUCCESS"

    time.sleep(0.1)
    assert manager.stats["http_req_count"] == 1
