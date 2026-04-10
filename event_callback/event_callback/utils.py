import functools
import logging
import os
import time
from logging.handlers import RotatingFileHandler
from typing import Any, Callable, Dict, Optional


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


class classproperty:
    """自定义描述符类，实现 classmethod + property 的效果"""

    def __init__(self, func):
        self.func = func  # 保存传入的类方法函数

    def __get__(self, instance, owner):
        # instance: 访问的实例（如果是类访问则为 None）
        # owner: 访问的类（无论实例/类访问，都能拿到类对象）
        return self.func(owner)  # 调用函数时传入类对象


import weakref


def throttle(frequency: float, max_calls: int = float("inf")):
    """
    外层函数：只负责接收节流参数
    """

    class ThrottleWrapper:
        """
        内层类：负责真正的装饰、描述符逻辑和状态管理
        """

        def __init__(self, func: Callable):
            self.func = func
            self.frequency = frequency
            self.max_calls = max_calls

            # 用于成员方法的实例隔离状态
            self.instance_states = weakref.WeakKeyDictionary()
            # 用于普通函数的全局状态
            self.func_state = {"last_called": 0.0, "call_count": 0}

            functools.update_wrapper(self, func)

        def _get_state(self, instance: Any) -> dict:
            if instance is None:
                return self.func_state
            if instance not in self.instance_states:
                self.instance_states[instance] = {"last_called": 0.0, "call_count": 0}
            return self.instance_states[instance]

        def _execute(self, instance: Any, *args, **kwargs) -> Any:
            """统一的节流执行逻辑"""
            state = self._get_state(instance)

            if state["call_count"] >= self.max_calls:
                return None
            current_time = time.time()
            if current_time - state["last_called"] >= 1 / self.frequency:
                result = self.func(*args, **kwargs)
                state["last_called"] = current_time
                state["call_count"] += 1
                return result
            return None

        def __call__(self, *args, **kwargs) -> Any:
            """
            当装饰的是普通函数时，调用会触发这里。
            注意：这里的 __call__ 只负责执行业务逻辑，不再负责接收 func！
            """
            return self._execute(None, *args, **kwargs)

        def __get__(self, instance: Any, owner: Any) -> Callable:
            """
            当装饰的是类成员方法时，通过 obj.method 访问会触发这里。
            """
            if instance is None:
                return self

            @functools.wraps(self.func)
            def bound_wrapper(*args, **kwargs):
                # 将 instance 传入，实现状态隔离，同时自动填补 self 参数
                return self._execute(instance, instance, *args, **kwargs)

            # 完美解决属性丢失问题：将状态暴露给绑定的方法
            bound_wrapper.last_called = lambda: self._get_state(instance)["last_called"]
            bound_wrapper.call_count = lambda: self._get_state(instance)["call_count"]

            return bound_wrapper

        # 为普通函数暴露状态
        def last_called(self):
            return self.func_state["last_called"]

        def call_count(self):
            return self.func_state["call_count"]

    # 返回这个类本身，@throttle()() 语法会自动实例化它并传入被装饰的函数
    return ThrottleWrapper


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
    from colorama import Fore, Style, init

    init(autoreset=True)  # 自动重置颜色，避免后续文字变色

    class ColoredFormatter(logging.Formatter):
        """自定义带颜色的日志格式器"""

        # 级别 -> 颜色映射
        COLOR_MAP = {
            logging.DEBUG: Fore.BLUE,
            logging.INFO: Fore.GREEN,
            logging.WARNING: Fore.YELLOW,
            logging.ERROR: Fore.RED,
            logging.CRITICAL: Fore.MAGENTA,
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
    LOG_FORMAT = "%(asctime)s [%(levelname)s] %(message)s"
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
