import copy
import functools
import inspect
import logging
import time
from concurrent.futures import Future, ThreadPoolExecutor
from functools import wraps
from typing import Any, Callable, List, TypeVar

logger = logging.getLogger(__name__)

# 定义泛型，用于保持被装饰函数的方法签名不丢失
F = TypeVar("F", bound=Callable[..., Any])


class BaseEvent:
    """事件描述符：支持完美的 IDE 类型提示与安全的类继承"""

    def __init__(self):
        # 预先声明属性，消除类型检查警告
        self.event_name: str = ""
        self.component_cls: type = None  # type: ignore

    def __set_name__(self, owner, name):
        self.event_name = name
        self.component_cls = owner

    def __get__(self, instance, owner):
        # 【核心魔法】：不要直接修改 self.component_cls (会导致多线程竞争或覆盖)
        # 而是浅拷贝一个自己。
        # 这样返回的对象依然是 BaseEvent（或其子类）的实例，IDE 能完美识别它的 __call__！
        bound_event = copy.copy(self)
        bound_event.component_cls = owner
        return bound_event

    # ==========================================
    # 在这里重载 __call__！
    # 你可以随心所欲地添加强类型提示，IDE 会完美补全
    # ==========================================
    def __call__(self, **kwargs) -> Callable[[F], F]:
        """
        给方法打标签
        :param route: 路由路径
        :param methods: HTTP 方法列表
        """

        def decorator(func: F) -> F:
            if not hasattr(func, "__target_events__"):
                func.__target_events__ = []
            func.__target_events__.append(
                {
                    "comp_cls": self.component_cls,  # 这里的 self 是克隆体，已经绑定了真实的 B
                    "event_name": self.event_name,
                    "params": kwargs,
                }
            )
            return func

        return decorator


def create_wrapper(original_cb):
    """
    制造一个工厂函数，完美避开 Python 循环闭包的晚绑定陷阱
    """

    @functools.wraps(original_cb)
    def wrapper(*w_args, **w_kwargs):
        try:
            # 1. 正常执行用户的回调（内部可能有耗时计算、触发新任务、rospy.pub等）
            return original_cb(*w_args, **w_kwargs)
        finally:
            # 2. 核心魔法：使用 finally 确保即使回调抛出异常，依然会让出 GIL
            # 只要回调执行完毕（代表 publish 已经塞入了数据），
            # 立刻强制交出执行权，给 rospy 留出 2~5 毫秒把数据发往网卡！
            time.sleep(0.005)

    return wrapper


class BaseComponent:
    def __init__(self):
        self._routes = []
        self.worker_pool = ThreadPoolExecutor()  # 回调执行线程池

    def bind_callback(self, event_name: str, params: dict, callback: Callable):
        # 把路由参数和回调函数存起来
        self._routes.append((event_name, params, callback))

    def bind_done(self):
        pass

    def block_trigger(self, event_name: str, match_params: dict, *args, **kwargs):
        """带参数过滤的触发器, 阻塞直到回调完成"""
        futures = self.trigger(event_name, match_params, *args, **kwargs)
        out = []
        for f in futures:
            out.append(f.result())
        return out

    def trigger(
        self, event_name: str, match_params: dict, *args, **kwargs
    ) -> List[Future]:
        """带参数过滤的触发器, 提交到一个线程池中执行"""
        out = []

        for evt, params, cb in self._routes:
            if evt == event_name:
                # 路由匹配逻辑
                if all(match_params.get(k) == v for k, v in params.items()):
                    # 使用工厂函数生成安全的包裹器
                    safe_cb = create_wrapper(cb)

                    # 提交到线程池
                    future = self.worker_pool.submit(safe_cb, *args, **kwargs)
                    out.append(future)

        return out


class BaseManager:
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)

        # 获取子类原始的 __init__
        original_init = cls.__init__

        # 定义包装器
        @wraps(original_init)
        def new_init(self, *args, **kwargs):
            # 1. 照常执行 __init__ (可能是当前类的，也可能是 super() 调上来的)
            original_init(self, *args, **kwargs)

            # 2. 核心魔法：判断当前包装的类 (cls) 是不是实例的真实类 (type(self))
            # 这样就能保证，只有最外层（最底层子类）的 __init__ 结束时，才会触发一次绑定
            if cls is type(self):
                self._bind_events()

        # 替换子类的 __init__
        cls.__init__ = new_init

    def __init__(self, *components):
        self._components_for_binding = components

    def _bind_events(self):
        """在这里执行你的扫描和绑定逻辑"""
        comp_map = {type(c): c for c in self._components_for_binding}
        for _, method in inspect.getmembers(self, callable):
            target_events = getattr(method, "__target_events__", [])
            for target in target_events:
                comp_cls = target["comp_cls"]
                if comp_cls in comp_map:
                    comp_map[comp_cls].bind_callback(
                        event_name=target["event_name"],
                        params=target["params"],
                        callback=method,
                    )
                else:
                    raise RuntimeError(
                        f"Callback {method.__name__} depends on {comp_cls.__name__}, "
                        "but it's not found in the manager's components!"
                    )
        for comp in comp_map.values():
            comp.bind_done()
