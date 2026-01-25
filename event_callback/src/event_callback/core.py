from abc import ABC, abstractmethod
import functools
from typing import Tuple, Type, Dict, List, Callable, Any, ClassVar, Final
from .types import T_BaseManager
import inspect

# 装饰器类型定义：支持编辑器类型推导
Decorator = Callable[..., Callable[[Callable], Callable]]
# 回调注册表项类型：(函数, 装饰器位置参数, 装饰器关键字参数)
CallbackItem = Tuple[Callable, List[Any], Dict[str, Any]]


def get_class_qualname(func: Callable):
    """
    从方法对象获取所属类的**限定名**（含嵌套层级，无模块）
    :param func: 成员方法对象（装饰/未装饰、绑定/未绑定均可）
    :return: 类的限定名字符串 / None（非成员方法）
    """
    # 补救：递归提取装饰器包装的原方法（处理未加@wraps的情况）
    while hasattr(func, "__wrapped__"):
        func = func.__wrapped__
    try:
        # 方法__qualname__格式：Class[.InnerClass].method → 分割取类部分
        func_qualname = func.__qualname__
        # 分割后去掉最后一个元素（方法名），剩余为类的限定名
        class_qualname_parts = func_qualname.split(".")[:-1]
        if not class_qualname_parts:
            return None  # 普通函数，无所属类
        return ".".join(class_qualname_parts)
    except AttributeError:
        return None  # 无__qualname__属性，非方法对象


def get_classname(func: Callable):
    """
    从方法对象获取所属类的**带模块完整路径名**（模块名.类限定名，全局唯一）
    :param func: 成员方法对象
    :return: 类的完整路径名字符串 / None（非成员方法）
    """
    # 先获取类的限定名
    class_qualname = get_class_qualname(func)
    if not class_qualname:
        return None
    # 补救：提取原方法
    while hasattr(func, "__wrapped__"):
        func = func.__wrapped__
    try:
        # 方法的__module__属性 → 所属模块名
        func_module = func.__module__
        # 拼接：模块名 + 类限定名
        return f"{func_module}.{class_qualname}"
    except AttributeError:
        return None


def get_classname_obj(obj: Any):
    cls = obj.__class__
    classname = f"{cls.__module__}.{cls.__qualname__}"
    return classname


class R:
    """
    全局组件注册器：提供组件注册接口、动态生成装饰器(R.http/R.ros)、实例回调批量注册
    单例设计，全局唯一组件注册入口，支持编辑器实时类型推导
    """

    # 单例实例
    _instance: ClassVar[Any] = None
    # 已注册组件：{组件名: 组件类} 如 {"http": HttpComponent, "ros": ROSComponent}
    _registered_comps: Final[Dict[str, Type["BaseComponent"]]] = {}
    # 装饰器临时注册表：{组件名: 回调项列表} 存储被R.xxx装饰的函数及参数
    _temp_callback_map: Final[Dict[str, Dict[str, List[CallbackItem]]]] = {}
    # 类型注解映射：用于编辑器实时类型推导，动态添加R.xxx的注解
    __annotations__: Dict[str, Decorator] = {}

    def __new__(cls):
        """单例模式：保证全局只有一个R实例"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def register_component(
        self, comp_name: str, comp_cls: Type["BaseComponent"]
    ) -> None:
        """
        注册组件核心接口：动态生成对应装饰器(R.xxx)，并添加类型推导
        :param comp_name: 组件别名（如http/ros，作为R的属性名）
        :param comp_cls: 组件类（必须继承BaseComponent）
        """
        if not issubclass(comp_cls, BaseComponent) or comp_cls == BaseComponent:
            raise TypeError(
                f"comp_cls must be subclass of BaseComponent, not {comp_cls}"
            )
        if comp_name in self._registered_comps:
            raise ValueError(f"Component {comp_name} already registered")

        # 注册组件类
        self._registered_comps[comp_name] = comp_cls
        # 初始化该组件的临时回调表
        self._temp_callback_map[comp_name] = []
        # 生成该组件的装饰器并绑定到R实例
        setattr(self, comp_name, self._create_comp_decorator(comp_name))
        # 动态添加类型注解：让编辑器识别R.xxx为装饰器类型，支持实时推导
        self.__annotations__[comp_name] = Decorator

    def _create_comp_decorator(self, comp_name: str) -> Decorator:
        """为指定组件生成装饰器，记录函数及装饰器参数到临时注册表"""

        def decorator(*args: Any, **kwargs: Any) -> Callable[[Callable], Callable]:
            def wrapper(func: Callable) -> Callable:
                class_name = get_classname(func)
                comp_map = self._temp_callback_map.get(class_name, {})
                callback_list = comp_map.get(comp_name, [])
                callback_list.append((func, args, kwargs))
                comp_map[comp_name] = callback_list
                self._temp_callback_map[class_name] = comp_map
                return func

            return wrapper

        return decorator


# 全局唯一R实例：外部所有操作均通过该实例进行
R = R()


class CallbackManager:
    """
    回调管理器基类：元类驱动，初始化组件实例，触发组件回调注册
    子类仅需继承 + 在__init__中指定component_config，组件由R全局注册
    """

    def __init__(self, component_config: Dict[str, Dict[str, Any]] = None):
        """
        :param component_config: 组件初始化配置 {组件名: {配置参数}, ...} 如 {"ros": {"node_name": "drone"}}
        """
        self.component_config = component_config or {}

        # 查找R中是否有需要实例化的compnent
        classname = get_classname_obj(self)
        for comp in R._temp_callback_map.get(classname, {}).keys():
            if comp not in self.component_config:
                self.component_config[comp] = {}

        self._component_instances: Dict[str, BaseComponent] = {}  # 组件实例映射：{组件名: 实例}
        self._init_all_components()  # 初始化R中已注册的组件
        self._register_all_components()  # 触发组件的回调注册逻辑

    def _init_all_components(self) -> None:
        """初始化R中已注册、且在component_config中配置的组件实例"""
        for comp_name, comp_cfg in self.component_config.items():
            comp_cls = R._registered_comps.get(comp_name)
            if comp_cls is None:
                raise KeyError(f"No compnent named: {comp_name}")
            # 初始化组件实例并绑定当前manager实例
            comp_instance = comp_cls(manager_instance=self, **comp_cfg)
            self._component_instances[comp_name] = comp_instance

    def _register_all_components(self) -> None:
        """遍历所有组件实例，触发其回调注册逻辑"""
        classname = get_classname_obj(self)
        for comp_instance in self._component_instances.values():
            callbacks = R._temp_callback_map.get(classname, {}).get(
                comp_instance.name, []
            )
            comp_instance.register_callbacks(callbacks)

    def get_component(self, name):
        return self._component_instances.get(name)

class BaseComponent(ABC):
    """组件抽象基类：所有组件的父类，强制绑定CallbackManager子类实例，提供回调列表获取"""

    name = "base"

    def __init_subclass__(cls):
        R.register_component(cls.name, cls)

    def __init__(self, manager_instance: T_BaseManager, **config: Any):
        self.manager_instance = manager_instance  # 绑定的manager实例
        self.config = config  # 组件初始化配置

    @abstractmethod
    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        """组件核心抽象方法：实现自身的回调注册逻辑（如ROS订阅/HTTP路由注册）"""
        raise NotImplementedError()
