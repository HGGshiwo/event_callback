from typing import Any, Dict

from fastapi import Request


async def request2dict(request: Request) -> Dict[str, Any]:
    """仅序列化 solve_dependencies 所需的核心字段"""
    request_data = {
        "method": request.method,
        "path": request.url.path,
        "query_params": dict(request.query_params),
        "body": (await request.body()).decode("utf-8"),
        "headers": dict(request.headers),
    }
    return request_data


def dict2route(route_params: Dict[str, Any]) -> Dict[str, Any]:
    """从 JSON 还原路由参数, 缺少endpoint"""
    # 构造 add_api_route 所需的最终参数
    add_route_params = {
        "path": route_params["path"],
        "methods": route_params["methods"],
        "summary": route_params["summary"],
        "description": route_params["description"],
        "openapi_extra": route_params["openapi_extra"],
        # 核心：还原参数注解对应的依赖（FastAPI 自动解析注解）
        # 无需手动构造 Depends，FastAPI 会根据 endpoint 的注解自动解析
    }
    return add_route_params
