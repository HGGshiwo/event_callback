import inspect
import urllib
from contextlib import AsyncExitStack
from typing import (
    Any,
    Callable,
    Dict,
    Optional,
    Tuple,
    Type,
    get_args,
    get_origin,
    get_type_hints,
)

from fastapi import Request
from fastapi.params import Body, Path, Query
from pydantic import BaseModel
from pydantic.json_schema import model_json_schema
from starlette.datastructures import Headers, QueryParams


def generate_openapi_extra(func: Callable) -> Dict[str, Any]:
    """
    解析函数注解，生成符合FastAPI openapi_extra规范的字典结构
    兼容场景：无Path/Query/Body装饰、简单类型注解、无注解、Pydantic模型注解

    :param func: 待解析的函数
    :return: 可直接传入add_api_route的openapi_extra字典
    """
    # 初始化openapi_extra基础结构
    summary, description = parse_docstring(func)
    openapi_extra = {
        "summary": summary,
        "description": description,
        "parameters": [],  # 存放Path/Query参数
        "responses": {
            "200": {
                "description": "请求成功",
                "content": {"application/json": {"schema": {"type": "object"}}},
            }
        },
    }

    # 存放Body参数（Pydantic模型/显式Body参数）
    body_params = {}

    # 解析函数签名和类型注解
    sig = inspect.signature(func)
    type_hints = get_type_hints(func)

    for param_name, param in sig.parameters.items():
        # 跳过self/cls等类方法参数（可选，根据你的场景调整）
        if param_name in ("self", "cls"):
            continue

        # 1. 基础参数信息初始化
        param_default = (
            param.default if param.default is not inspect.Parameter.empty else None
        )
        param_source = None  # Path/Query/Body/auto（auto表示自动推断）
        default_value = None
        is_required = True  # 默认必填

        # 2. 处理默认值和显式参数来源（Path/Query/Body）
        if param_default is not None:
            # 显式指定了Path/Query/Body
            if isinstance(param_default, (Path, Query, Body)):
                if isinstance(param_default, Path):
                    param_source = "Path"
                elif isinstance(param_default, Query):
                    param_source = "Query"
                elif isinstance(param_default, Body):
                    param_source = "Body"
                # 提取FastAPI参数对象的默认值和必填性
                default_value = (
                    param_default.default if param_default.default is not ... else None
                )
                is_required = (
                    param_default.required
                    if hasattr(param_default, "required")
                    else (default_value is None)
                )
            else:
                # 普通默认值（无FastAPI装饰）
                default_value = param_default
                is_required = False  # 有默认值则非必填
        else:
            # 无默认值 → 必填
            is_required = True

        # 3. 处理类型注解，生成OpenAPI Schema
        openapi_schema = {}
        anno = type_hints.get(param_name)
        is_pydantic_model = False  # 是否是Pydantic模型

        if anno is not None:
            # 处理Optional类型（如Optional[int] → int + nullable=True）
            if get_origin(anno) is Optional:
                base_type = get_args(anno)[0]
                openapi_schema["nullable"] = True
                anno = base_type

            # 基础类型映射（OpenAPI规范）
            basic_type_map = {
                int: "integer",
                str: "string",
                bool: "boolean",
                float: "number",
                list: "array",
                dict: "object",
                type(None): "null",
            }
            # Pydantic模型
            if isinstance(anno, Type) and issubclass(anno, BaseModel):
                openapi_schema = model_json_schema(anno)
                openapi_schema["description"] = f"Pydantic模型：{anno.__name__}"
                is_pydantic_model = True
            # 基础类型
            elif anno in basic_type_map:
                openapi_schema["type"] = basic_type_map[anno]
                openapi_schema["description"] = f"类型：{anno.__name__}"
            # 其他未知类型（默认string）
            else:
                openapi_schema["type"] = "string"
                openapi_schema["description"] = f"未知类型：{str(anno)}"
        else:
            # 无类型注解 → 默认string类型
            openapi_schema["type"] = "string"
            openapi_schema["description"] = "无类型注解"

        # 4. 添加默认值到schema
        if default_value is not None:
            openapi_schema["default"] = default_value

        # 5. 自动推断参数来源（无显式Path/Query/Body时）
        if param_source is None:
            # 规则：Pydantic模型→Body，其他→Query
            if is_pydantic_model:
                param_source = "Body"
            else:
                param_source = "Query"

        # 6. 根据参数来源组装到openapi_extra
        if param_source == "Path":
            openapi_extra["parameters"].append(
                {
                    "name": param_name,
                    "in": "path",
                    "required": True,  # Path参数必须必填
                    "schema": openapi_schema,
                    "description": f"路径参数：{param_name}",
                }
            )
        elif param_source == "Query":
            openapi_extra["parameters"].append(
                {
                    "name": param_name,
                    "in": "query",
                    "required": is_required,
                    "schema": openapi_schema,
                    "description": f"查询参数：{param_name}",
                }
            )
        elif param_source == "Body":
            # 处理Body参数（支持单个/多个Pydantic模型）
            body_params[param_name] = openapi_schema
            openapi_extra["requestBody"] = {
                "content": {
                    "application/json": {
                        "schema": {
                            "type": "object",
                            "properties": body_params,
                            "required": [
                                k for k, v in body_params.items() if is_required
                            ],
                        }
                    }
                },
                "required": len(body_params) > 0,
            }

    return openapi_extra


def parse_docstring(func: Callable) -> Tuple[str, str]:
    """解析函数文档字符串，返回 {summary, description}（和 FastAPI 逻辑一致）"""
    doc = func.__doc__ or ""
    if not doc:
        summary = f"调用函数: {func.__name__}"  # 函数名作为摘要
        description = f"自动生成的{func.__name__}接口文档"
        return summary, description

    # 步骤1：去除所有行的通用缩进（FastAPI 核心逻辑）
    lines = doc.expandtabs().splitlines()
    # 找到非空行的最小缩进
    min_indent = float("inf")
    for line in lines[1:]:  # 跳过第一行（summary）
        stripped = line.lstrip()
        if stripped:
            indent = len(line) - len(stripped)
            min_indent = min(min_indent, indent)
    # 去除通用缩进
    if min_indent != float("inf"):
        lines = [lines[0]] + [line[min_indent:] for line in lines[1:]]

    # 步骤2：分割 summary（第一行）和 description（剩余内容）
    summary = lines[0].strip()
    description = "\n".join(line.rstrip() for line in lines[1:]).strip()

    return summary, description


def route2dict(url: str, method: str, func: Callable) -> Dict[str, Any]:
    """构造路由注册参数（add_api_route 所需）"""
    # 解析函数注解
    openapi_extra = generate_openapi_extra(func)
    # 构造核心路由参数
    summary, description = parse_docstring(func)
    route_params = {
        "path": url,
        "methods": [method.upper()],  # 统一转大写（GET/POST/PUT等）
        "openapi_extra": openapi_extra,  # 函数注解元数据（核心）
        "summary": summary,
        "description": description,
    }
    return route_params


async def dict2request(request_data: Dict[str, Any]) -> Request:
    """还原仅满足 solve_dependencies 解析的 Request 对象（无 app 等非核心字段）"""

    # 1. 重构核心 scope（仅保留 solve_dependencies 必需的字段）
    query_string = urllib.parse.urlencode(request_data["query_params"]).encode("utf-8")
    headers = Headers(request_data["headers"])

    scope = {
        "type": "http",  # 固定为 http（必需）
        "method": request_data["method"],  # HTTP 方法（必需）
        "path": request_data["path"],  # 路径（必需）
        "query_string": query_string,  # 查询参数（必需）
        "headers": headers.raw,  # Headers（必需，影响 Content-Type 解析）
        # 非必需字段用空值/默认值填充（避免 solve_dependencies 报错）
        "fastapi_function_astack": AsyncExitStack(),
        "fastapi_inner_astack": AsyncExitStack(),
        "app": object(),  # 用空对象替代真实 app 实例
        "raw_path": b"",
        "scheme": "http",
        "client": None,
        "server": None,
        "root_path": "",
        "extensions": {},
    }

    # 2. 重构 receive 函数（仅返回请求体，必需）
    async def receive():
        return {
            "type": "http.request",
            "body": request_data["body"].encode("utf-8"),
            "more_body": False,
        }

    # 3. 实例化简化版 Request
    restored_request = Request(scope=scope, receive=receive)
    # 补充查询参数对象（可选，solve_dependencies 也能从 scope 解析）
    restored_request._query_params = QueryParams(query_string)
    return restored_request
