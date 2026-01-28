from abc import ABC, abstractmethod
from types import MethodType
from typing import (
    Optional,
    Type,
    Dict,
    List,
    Callable,
    Any,
    ClassVar,
    Final,
    TypeVar,
)

from event_callback.utils import get_classname
from event_callback.types import CallbackItem, Decorator


class Registery:
    """
    全局组件注册器：提供组件注册接口、动态生成装饰器(R.http/R.ros)、实例回调批量注册
    单例设计，全局唯一组件注册入口，支持编辑器实时类型推导
    """

    # 单例实例
    _instance: ClassVar[Any] = None
    # 装饰器注册表：manager_name -> compnent_name -> (callback, ...args)
    _callback_map: Final[Dict[str, Dict[str, List[CallbackItem]]]] = {}
    # 组件注册表: classname -> comp_class
    _component_map: Dict[str, Type["BaseComponent"]] = {}

    def __new__(cls):
        """单例模式：保证全局只有一个R实例"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def _create_comp_decorator(
        self, comp_cls: Type["BaseComponent"], *args: Any, **kwargs: Any
    ) -> Decorator:
        """为指定组件生成装饰器，记录函数及装饰器参数到临时注册表"""
        comp_name = get_classname(comp_cls, False)
        if comp_name not in self._component_map:
            self._component_map[comp_name] = comp_cls

        def wrapper(func: Callable) -> Callable:
            class_name = get_classname(func, True)
            comp_map = self._callback_map.get(class_name, {})
            callback_list = comp_map.get(comp_name, [])
            callback_list.append((func, args, kwargs))
            comp_map[comp_name] = callback_list
            self._callback_map[class_name] = comp_map
            return func

        return wrapper


# 全局唯一R实例：外部所有操作均通过该实例进行
R: Registery = Registery()

T = TypeVar("T")


class CallbackMixin:
    """
    1. 和Manager共享组件的实例，组件初始化优先级: manager config > mixin config > 没有明确配置参数
    2. 允许绑定回调函数，回调函数的self指向Mixin
    """

    def __init__(self, component_config: List[Any] = None):
        component_config = component_config or []
        self.component_config = {
            get_classname(config[0], False): (self, config[0], config[1])
            for config in component_config
        }
        self._component_instances: Dict[str, BaseComponent] = {}
        self._update_component_config()

    def _update_component_config(self) -> None:
        """查找R中是否有需要被实例化的组件，这些组件的初始化参数默认为None"""
        classname = get_classname(self, False)
        for comp_name in R._callback_map.get(classname, {}).keys():
            if comp_name not in self.component_config:
                self.component_config[comp_name] = (
                    self,
                    R._component_map[comp_name],
                    None,
                )

    def get_component_instance(self, comp_cls: Type["T"]) -> T:
        comp_name = get_classname(comp_cls, False)
        return self._component_instances.get(comp_name)


class CallbackManager(CallbackMixin):
    """
    回调管理器基类：元类驱动，初始化组件实例，触发组件回调注册
    子类仅需继承 + 在__init__中指定component_config，组件由R全局注册
    """

    def __init__(
        self,
        component_config: List[Any] = None,
        mixins: Optional[List[CallbackMixin]] = None,
    ):
        """
        :param list component_config: 组件初始化配置 {组件名: {配置参数}, ...} 如 {"ros": {"node_name": "drone"}}
        :param CallbackMixin mixins: 允许注册组件，但是不实例化组件
        """
        super().__init__(component_config)
        self.mixins = mixins if mixins is not None else []
        for mixin in self.mixins:
            mixin._update_component_config()
        self._init_all_components()  # 初始化已注册的组件，并注册回调

    def _init_all_components(self) -> None:
        """初始化在或者回调中被使用或者component_config中额外指定的组件实例"""
        # 检查是否有重复初始化的组件，只检查手动指定的，不检查自动注册的
        for mixin in self.mixins:
            for comp_name, comp_cfg in mixin.component_config.items():
                if comp_name not in self.component_config:
                    self.component_config[comp_name] = comp_cfg
                    continue
                print(
                    f"Wraning: {get_classname(mixin)} has duplicate component config for {comp_name}"
                    f"with {get_classname(self.component_config[comp_name][0])}, skip the config"
                )

        for comp_name, comp_cfg in self.component_config.items():
            # 初始化组件实例并绑定当前manager实例
            try:
                (manager_cls, comp_cls, comp_kwargs) = comp_cfg
            except Exception as e:
                print(
                    f"Error in component: {comp_name}, ComponentHelper.config must return 2 items: [ComponentClass, kwargs]"
                )
                raise e
            comp_kwargs = {} if comp_kwargs is None else comp_kwargs
            comp_instance = comp_cls(**comp_kwargs)
            self._component_instances[comp_name] = comp_instance
            # 直接进行注册
            classname = get_classname(manager_cls, False)
            callbacks = R._callback_map.get(classname, {}).get(comp_name, [])
            callbacks = [
                (MethodType(callback[0], manager_cls), *callback[1:])
                for callback in callbacks
            ]
            comp_instance.register_callbacks(callbacks)

        # 所有的manager+mixin共享component
        for mixin in self.mixins:
            mixin._component_instances = self._component_instances


class BaseComponent(ABC):
    """组件抽象基类：所有组件的父类，强制绑定CallbackManager子类实例，提供回调列表获取"""

    def __init__(self, **config: Any):
        self.config = config  # 组件初始化配置

    @abstractmethod
    def register_callbacks(self, callbacks: List[CallbackItem]) -> None:
        """组件核心抽象方法：实现自身的回调注册逻辑（如ROS订阅/HTTP路由注册）"""
        raise NotImplementedError()


class BaseComponentHelper(ABC):
    """组件辅助抽象基类"""

    # target: Type["BaseComponent"]

    @classmethod
    def config(cls, *args, **kwargs):
        raise NotImplementedError()

    @classmethod
    def get_component_instance(cls, manager: CallbackManager):
        return manager.get_component_instance(cls.target)
