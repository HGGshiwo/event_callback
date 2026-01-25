from typing import TypeVar

# 约束为BaseManager的子类实例
T_BaseManager = TypeVar('T_BaseManager', bound='BaseManager')