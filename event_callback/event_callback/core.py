import inspect
import logging
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, TypeVar

logger = logging.getLogger(__name__)

# 定义泛型，用于保持被装饰函数的方法签名不丢失
F = TypeVar("F", bound=Callable[..., Any])


class BaseEvent:
    """事件描述符基类：负责把装饰器的参数变成元数据，贴到 Manager 的方法上"""

    def __set_name__(self, owner, name):
        # Python 魔法：自动获取自己被赋给的变量名 (例如 'on_request')
        self.component_cls = owner
        self.event_name = name

    def _mark_method(self, func: F, **route_params) -> F:
        """内部工具：给方法打标签"""
        if not hasattr(func, "__target_events__"):
            func.__target_events__ = []

        # 将 组件类、事件名、以及【装饰器传入的参数】一并存下来
        func.__target_events__.append(
            {
                "comp_cls": self.component_cls,
                "event_name": self.event_name,
                "params": route_params,
            }
        )
        return func

    def __call__(self):
        def decorator(func: F):
            return self._mark_method(func)

        return decorator


class BaseComponent:
    def __init__(self):
        self._routes = []
        self.worker_pool = ThreadPoolExecutor(max_workers=10)  # 回调执行线程池

    def bind_callback(self, event_name: str, params: dict, callback: Callable):
        # 把路由参数和回调函数存起来
        self._routes.append((event_name, params, callback))

    def safe_excute_cb(self, cb, *args, **kwargs):
        try:
            cb(*args, **kwargs)
        except Exception as e:
            import traceback

            logger.error(traceback.format_exc())

    def trigger(self, event_name: str, match_params: dict, *args, **kwargs):
        """带参数过滤的触发器, 提交到一个线程池中执行"""
        for evt, params, cb in self._routes:
            if evt == event_name:
                # 路由匹配逻辑：如果装饰器上定义的参数，和当前事件的参数匹配，则执行回调
                # 例如：装饰器定义了 path="/api"，当前触发也是 path="/api"
                if all(match_params.get(k) == v for k, v in params.items()):
                    self.worker_pool.submit(self.safe_excute_cb, cb, *args, **kwargs)


class BaseManager:
    def __init__(self, *components: BaseComponent):
        comp_map = {type(c): c for c in components}

        # 扫描 Manager 上的所有方法，提取元数据
        for _, method in inspect.getmembers(self, inspect.ismethod):
            target_events = getattr(method, "__target_events__", [])
            for target in target_events:
                comp_cls = target["comp_cls"]
                if comp_cls in comp_map:
                    # 将方法以及它带的参数，注册到 Component 实例中
                    comp_map[comp_cls].bind_callback(
                        event_name=target["event_name"],
                        params=target["params"],
                        callback=method,
                    )
                else:
                    raise RuntimeError(
                        f"Callback {method.__name__} depends on component {comp_cls.__name__}, "
                        "but that component not found in component list, make sure you pass it to the manager!"
                    )
