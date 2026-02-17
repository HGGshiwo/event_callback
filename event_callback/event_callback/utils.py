from contextlib import AsyncExitStack
import copy
import inspect
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import sys
import time
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
from pydantic import BaseModel
from pydantic.json_schema import model_json_schema
from starlette.datastructures import Headers, QueryParams
from fastapi import Request
from fastapi.params import Body, Query, Path
import urllib
from rosgraph.masterapi import Master


def get_class_qualname(obj: Any, is_method: bool) -> str:
    """
    从【方法对象/对象实例/类对象】获取所属类的限定名（含嵌套层级，无模块）

    :param obj: 成员方法（装饰/未装饰、绑定/未绑定）/ 类的实例 / 类对象
    :param is_method: 是成员方法而不是对象/类
    :return: 类的限定名字符串 / None（非方法/非实例/非类）
    """
    # 步骤1：处理装饰器包装的方法，递归提取原方法（保留原逻辑）
    func = obj
    while hasattr(func, "__wrapped__"):
        func = func.__wrapped__

    try:
        # 情况1：入参是【方法对象】→ 保留原解析逻辑（类.方法 → 截取类）
        if is_method:
            func_qualname = func.__qualname__
            class_parts = func_qualname.split(".")[:-1]
            if not class_parts:
                return None
            return ".".join(class_parts)
        # 情况2：入参是【对象实例/类对象】→ 直接取类的__qualname__
        else:
            # 实例→先获取所属类，类对象→直接使用
            cls = func.__class__ if not hasattr(func, "__qualname__") else func
            return cls.__qualname__
    except AttributeError:
        return None  # 无关键属性，返回None


def get_classname(obj: Any, is_method: bool) -> str:
    """
    从【方法对象/对象实例/类对象/MethodType绑定的函数】获取类的全局唯一名称（模块名.类限定名）

    :param obj: 成员方法 / 类的实例 / 类对象
    :param is_method: 是成员方法而不是对象/类
    :return: 类的完整路径名 / None（非方法/非实例/非类）
    """
    # 提取原方法/原对象的模块名（保留装饰器补救逻辑）
    if is_method:
        while hasattr(obj, "__wrapped__"):
            obj = obj.__wrapped__

        if hasattr(obj, "__self__"):
            # 如果有self, 直接获取__self__
            obj = obj.__self__
            is_method = False

    # 先获取类的限定名
    class_qualname = get_class_qualname(obj, is_method)
    if not class_qualname:
        return None

    try:
        # 方法对象→取方法的__module__；实例/类→取类的__module__
        if is_method:
            module = obj.__module__
        else:
            cls = obj.__class__ if not hasattr(obj, "__qualname__") else obj
            module = cls.__module__
        # 拼接全局唯一名称
        return f"{module}.{class_qualname}"
    except AttributeError:
        return None


try:
    import rospy
except:
    rospy = None


def rospy_is_shutdown():
    return rospy is not None and rospy.is_shutdown()


def rospy_init_node(node_name: str, anonymous: bool = False):
    """初始化ros node, 避免ros设置logger"""
    if rospy.core.is_initialized():
        return
    _logger = logging.getLogger()
    _handler = [h for h in _logger.handlers]
    _level = _logger.level
    rospy.init_node(node_name, anonymous=anonymous)
    logger = logging.getLogger()
    logger.setLevel(_level)
    for handler in _handler:
        logger.addHandler(handler)


class EnumMeta(type):
    """自定义枚举元类，实现可继承的枚举核心逻辑"""

    def __new__(cls, name, bases, attrs):
        # 收集所有枚举成员（排除特殊方法/属性，如__module__、__doc__等）
        enum_members = {}
        # 先合并父类的枚举成员（实现继承）
        for base in bases:
            if hasattr(base, "_members"):
                enum_members.update(base._members)

        # 收集当前类的枚举成员，去重（子类成员覆盖父类同名成员）
        for attr_name, attr_value in attrs.items():
            # 排除以下划线开头的特殊属性/方法
            if not attr_name.startswith("_"):
                # 检查值是否重复（保证枚举值唯一性）
                if attr_value in enum_members.values():
                    raise ValueError(f"枚举值 {attr_value} 已存在，无法重复定义")
                enum_members[attr_name] = attr_value

        # 创建枚举类实例
        enum_class = super().__new__(cls, name, bases, attrs)
        # 存储所有枚举成员（名称->值）
        enum_class._members = enum_members
        # 反向映射（值->名称），用于通过值查找成员
        enum_class._value_to_name = {v: k for k, v in enum_members.items()}

        # 为枚举类动态添加成员属性（不可变）
        for member_name, member_value in enum_members.items():
            # 封装枚举成员为实例，包含name和value属性
            member = enum_class._create_member(member_name, member_value)
            setattr(enum_class, member_name, member)

        return enum_class

    @staticmethod
    def _create_member(name, value):
        """创建枚举成员实例，保证不可变"""

        class EnumMember:
            __slots__ = ("name", "value")  # 限制属性，提升性能且不可动态添加属性

            def __init__(self, name, value):
                super().__setattr__("name", name)
                super().__setattr__("value", value)

            def __repr__(self):
                return f"<{self.name}: {self.value}>"

            def __eq__(self, other):
                if isinstance(other, EnumMember):
                    return self.value == other.value
                return self.value == other

            def __hash__(self):
                return hash(self.value)

            # 阻止修改属性，保证不可变性
            def __setattr__(self, key, value):
                if key in self.__slots__:
                    raise AttributeError("枚举成员属性不可修改")
                super().__setattr__(key, value)

        return EnumMember(name, value)


class BaseEnum(metaclass=EnumMeta):
    """可继承的基础枚举类，封装核心访问方法"""

    _members = {}  # 存储：名称 -> 值
    _value_to_name = {}  # 存储：值 -> 名称

    @classmethod
    def __call__(cls, value):
        """模拟原生Enum：通过值获取成员（如 Color(1)）"""
        member = cls.get(value)
        if not member:
            raise ValueError(f"{value} 不是 {cls.__name__} 的有效枚举值")
        return member

    @classmethod
    def __getitem__(cls, name):
        """通过名称获取成员（如 Color['RED']）"""
        if name not in cls._members:
            raise KeyError(f"{name} 不是 {cls.__name__} 的有效枚举名称")
        return getattr(cls, name)


class classproperty:
    """自定义描述符类，实现 classmethod + property 的效果"""

    def __init__(self, func):
        self.func = func  # 保存传入的类方法函数

    def __get__(self, instance, owner):
        # instance: 访问的实例（如果是类访问则为 None）
        # owner: 访问的类（无论实例/类访问，都能拿到类对象）
        return self.func(owner)  # 调用函数时传入类对象


def throttle(
    frequency: float, max_calls: int = float("inf")
) -> Callable[[Callable], Callable]:
    """
    创建一个节流装饰器，限制函数的调用频率和最大调用次数

    Args:
        frequency: 调用频率，例如 2.0 表示每秒最多调用 2 次
        max_calls: 最大调用次数，默认无限制

    Returns:
        装饰器函数
    """

    def decorator(func: Callable) -> Callable:
        # 初始化节流控制变量
        last_called = 0.0  # 上一次调用的时间戳
        call_count = 0  # 已调用次数

        def wrapper(*args: Any, **kwargs: Any) -> Any:
            nonlocal last_called, call_count

            # 检查是否达到最大调用次数
            if call_count >= max_calls:
                return None

            # 获取当前时间
            current_time = time.time()

            # 检查时间间隔是否满足节流要求
            if current_time - last_called >= 1 / frequency:
                try:
                    # 执行目标函数并记录结果
                    result = func(*args, **kwargs)
                    # 更新调用时间和计数
                    last_called = current_time
                    call_count += 1
                    return result
                except Exception as e:
                    # 保留原函数的异常
                    raise e

        # 暴露内部状态，方便调试
        wrapper.last_called = lambda: last_called
        wrapper.call_count = lambda: call_count

        return wrapper

    return decorator


class rostopic_field:
    """自动收集topic对应的数据"""

    def __init__(
        self,
        topic_name: str,
        topic_type: Type,
        timeout: Optional[float] = 0.01,
        format: Optional[Callable] = None,
    ):
        """
        Args:
            topic_name (str): ros topic名称
            topic_type (Type): ros topic 类型
            timeout (float, optional): 消息超时时间，访问超时的消息返回None，单位seconds. Defaults to 0.01.
            format (Callable, optional): 格式化函数，对获取的msg进行自定义格式化
        """
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._timeout = timeout
        self._msg = None
        self._format = format
        self._check_topic_type(topic_name, topic_type)
        # 保存订阅者引用，避免被GC回收导致订阅失效
        self._subscriber = rospy.Subscriber(topic_name, topic_type, self._msg_callback)

    def _get_topic_type(self, topic_name: str):
        """
        查询指定话题的类型
        :param topic_name: 话题名称（如 "/chatter"）
        :return: 话题类型字符串（如 "std_msgs/String"），若话题不存在返回None
        """
        try:
            master = Master(rospy.get_name())
            # 获取话题的所有连接信息
            topic_info = master.getTopicTypes()
            # 遍历查找目标话题
            for name, type_str in topic_info:
                if name == topic_name:
                    return type_str
            return None
        except Exception as e:
            rospy.logerr(f"查询话题类型失败: {e}")
            return None

    def _check_topic_type(self, topic_name: str, expected_type: Any):
        """
        带类型检查的订阅函数
        :param topic_name: 要订阅的话题名
        :param expected_type: 期望的消息类型类（如 String）
        """
        # 步骤1：获取期望类型的字符串表示（如 std_msgs/String）
        expected_type_str = expected_type._type

        # 步骤2：查询话题真实类型
        real_type_str = self._get_topic_type(topic_name)

        # 步骤3：类型检查与警告
        if real_type_str is None:
            # rospy.logwarn(f"警告：话题 {topic_name} 未找到，可能尚未发布！")
            return
        elif real_type_str != expected_type_str:
            raise ValueError(
                f"topic_name type dismatch! expected: {expected_type_str}, got: {real_type_str}"
            )

    def _msg_callback(self, msg: Any):
        """话题回调函数，更新最新消息"""
        self._msg = msg

    def _check_timeout(self) -> bool:
        """检查消息是否超时，超时返回True，未超时返回False"""
        if self._timeout is None:
            return False
        if not hasattr(self._msg, "header"):
            return False
        # 计算当前时间与消息时间的差值（秒）
        time_diff = (rospy.Time.now() - self._msg.header.stamp).to_sec()
        # 差值 >= 超时时间 → 超时
        return time_diff >= self._timeout

    @property
    def value(self):
        """获取处理后的话题值（含超时检查+格式化）"""
        if self._msg is None:
            return None
        if self._check_timeout():
            return None
        # 应用格式化函数（如果有）
        if self._format is not None:
            return self._format(self._msg)
        return self._msg


class rosparam_field:  # 修正拼写错误：filed → field
    """ROS参数操作类（get/set）"""

    def __init__(self, param_name: str, default: Optional[Any] = None):
        self._param_name = param_name
        self._default = default

    def get(self):
        """获取参数值，不存在返回None"""
        return rospy.get_param(self._param_name, self._default)

    def set(self, value: Any):
        """设置参数值"""
        rospy.set_param(self._param_name, value)


class ROSProxy:
    """ROS话题/参数代理类，简化属性访问"""

    def __getattribute__(self, name: str):
        # 调用父类方法安全获取属性，避免递归
        attr = super().__getattribute__(name)
        # 如果是话题字段，返回其处理后的值
        if isinstance(attr, rostopic_field):
            return attr.value
        # 如果是参数字段，返回其值
        if isinstance(attr, rosparam_field):
            return attr.get()
        # 普通属性直接返回
        return attr

    def __setattr__(self, name: str, value: Any):
        try:
            # 安全获取已存在的属性（避免触发__getattribute__的递归）
            attr = object.__getattribute__(self, name)
            # 如果是参数字段，调用set方法
            if isinstance(attr, rosparam_field):
                attr.set(value)
                return
        except AttributeError:
            # 属性不存在，直接设置
            pass
        # 普通属性/不存在的属性，调用父类方法设置，避免递归
        object.__setattr__(self, name, value)


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
        "fastapi_innner_astack": AsyncExitStack(),
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


def dict2route(route_params: Dict[str, Any]) -> Dict[str, Any]:
    """从 JSON 还原路由参数, 缺少endpoint"""
    # 构造 add_api_route 所需的最终参数
    add_route_params = {
        "path": route_params["path"],
        "methods": route_params["methods"],
        "summary": route_params["summary"],
        "description": route_params["description"],
        "openapi_extra": route_params["openapi_extra"]
        # 核心：还原参数注解对应的依赖（FastAPI 自动解析注解）
        # 无需手动构造 Depends，FastAPI 会根据 endpoint 的注解自动解析
    }
    return add_route_params


def setup_logger(
    log_dir: Optional[str] = None,
    log_level: str = "INFO",
    max_file_size_mb: int = 10,
    backup_count: int = 5,
) -> None:
    """
    初始化全局日志配置工具函数

    :param log_dir: 日志文件存储目录，传None则仅终端输出，传路径则添加文件Handler
    :param log_level: 全局日志级别，可选：DEBUG/INFO/WARNING/ERROR/CRITICAL
    :param max_file_size_mb: 单个日志文件最大大小（MB），仅log_dir有效时生效
    :param backup_count: 日志文件备份数量，仅log_dir有效时生效
    """
    from colorama import init, Fore, Style
    init(autoreset=True)  # 自动重置颜色，避免后续文字变色
    
    class ColoredFormatter(logging.Formatter):
        """自定义带颜色的日志格式器"""
        # 级别 -> 颜色映射
        COLOR_MAP = {
            logging.DEBUG: Fore.BLUE,
            logging.INFO: Fore.GREEN,
            logging.WARNING: Fore.YELLOW,
            logging.ERROR: Fore.RED,
            logging.CRITICAL: Fore.MAGENTA
        }

        def format(self, record):
            # 1. 给级别名称添加颜色
            record_copy = logging.makeLogRecord(record.__dict__)
            level_name = record.levelname
            color = self.COLOR_MAP.get(record.levelno, Fore.RESET)
            record_copy.levelname = f"{color}{level_name}{Style.RESET_ALL}"
            
            # 2. 保留原始格式（包含文件名、行号、日期等）
            return super().format(record_copy)
    
    # 1. 定义日志格式（包含文件名、行号、日期、级别、日志器名称、消息）
    LOG_FORMAT = (
        "%(asctime)s [%(levelname)s] %(message)s"
    )
    DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
    formatter = ColoredFormatter(LOG_FORMAT, datefmt=DATE_FORMAT)

    # 2. 获取并重置根日志器（避免重复配置）
    root_logger = logging.getLogger()
    # 清空现有Handler，防止重复输出
    root_logger.handlers.clear()
    # 转换日志级别字符串为logging常量
    level_mapping: Dict[str, int] = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL,
    }
    root_level = level_mapping.get(log_level.upper(), logging.INFO)
    root_logger.setLevel(root_level)

    # 3. 添加控制台Handler（始终启用）
    console_handler = logging.StreamHandler()
    console_handler.setLevel(root_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # 4. 若传入log_dir，添加文件Handler
    print(f"log_dir: {log_dir}")
    if log_dir is not None:
        # 确保日志目录存在
        os.makedirs(log_dir, exist_ok=True)
        log_file_path = os.path.join(log_dir, "app.log")
        # 按大小分割日志文件
        file_handler = RotatingFileHandler(
            filename=log_file_path,
            maxBytes=max_file_size_mb * 1024 * 1024,  # 转换为字节
            backupCount=backup_count,
            encoding="utf-8",  # 避免中文乱码
            mode="a",  # 追加模式
        )
        file_formatter = logging.Formatter(LOG_FORMAT, datefmt=DATE_FORMAT)
        file_handler.setLevel(root_level)
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    # 禁用日志向上传播（防止重复输出）
    # root_logger.propagate = False


def get_all_loggers():
    """
    获取所有已创建的 logger（包括 root 和子 logger）
    返回：字典，key=logger名称，value=logger对象
    """
    # 1. 获取 logging 管理器（存储所有 logger 的核心）
    logger_manager = logging.Logger.manager

    # 2. 获取所有子 logger（loggerDict 是 {名称: logger对象} 的字典）
    child_loggers = logger_manager.loggerDict

    # 3. 加入 root logger（名称固定为 ""）
    all_loggers = {"": logger_manager.root}

    # 4. 合并子 logger（过滤掉非 Logger 类型的占位符，避免异常）
    for name, logger in child_loggers.items():
        if isinstance(logger, logging.Logger):
            all_loggers[name] = logger

    return all_loggers


def print_logger_info(logger_name: str = None):
    """
    打印指定/所有 logger 的详细信息
    :param logger_name: 指定 logger 名称（None 则打印所有）
    """
    all_loggers = get_all_loggers()

    # 筛选要打印的 logger
    target_loggers = (
        all_loggers if logger_name is None else {logger_name: all_loggers[logger_name]}
    )

    print("=" * 80)
    for name, logger in target_loggers.items():
        # 基础信息
        logger_name_display = "root logger" if name == "" else f"子 logger [{name}]"
        print(f"\n{logger_name_display} 详细信息：")
        print(f"  - 自身级别：{logging.getLevelName(logger.level)} ({logger.level})")
        print(f"  - 有效级别：{logging.getLevelName(logger.getEffectiveLevel())}")
        print(f"  - propagate：{logger.propagate}")
        print(f"  - Handler 数量：{len(logger.handlers)}")
        print(f"  - 父 logger：{logger.parent.name if logger.parent else '无'}")

        # 列出 Handler（如果有）
        if logger.handlers:
            print("  - 已配置 Handler：")
            for idx, handler in enumerate(logger.handlers):
                print(
                    f"    [{idx}] {type(handler).__name__} (级别：{logging.getLevelName(handler.level)})"
                )
    print("\n" + "=" * 80)
