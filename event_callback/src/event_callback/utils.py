from typing import Any


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

