from typing import Any, Callable, Dict, List, Tuple, TypeVar

# 装饰器类型定义：支持编辑器类型推导
Decorator = Callable[..., Callable[[Callable], Callable]]
# 回调注册表项类型：(函数, 装饰器位置参数, 装饰器关键字参数)
CallbackItem = Tuple[Callable, List[Any], Dict[str, Any]]
