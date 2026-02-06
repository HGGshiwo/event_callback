import time
from typing import Any, Callable


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
    从【方法对象/对象实例/类对象】获取类的全局唯一名称（模块名.类限定名）

    :param obj: 成员方法 / 类的实例 / 类对象
    :param is_method: 是成员方法而不是对象/类
    :return: 类的完整路径名 / None（非方法/非实例/非类）
    """
    # 先获取类的限定名
    class_qualname = get_class_qualname(obj, is_method)
    if not class_qualname:
        return None

    # 提取原方法/原对象的模块名（保留装饰器补救逻辑）
    func = obj
    while hasattr(func, "__wrapped__"):
        func = func.__wrapped__

    try:
        # 方法对象→取方法的__module__；实例/类→取类的__module__
        if is_method:
            module = func.__module__
        else:
            cls = func.__class__ if not hasattr(func, "__qualname__") else func
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
        frequency: 最小调用间隔（秒），例如 1.0 表示每秒最多调用 1 次
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
            if current_time - last_called >= frequency:
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
