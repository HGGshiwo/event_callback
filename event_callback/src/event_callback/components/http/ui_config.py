from dataclasses import asdict, dataclass, field, fields
from inspect import isclass
from typing import Any, Dict, Type
import uuid
from event_callback.utils import BaseEnum, get_classname
import numpy as np


class ButtonType(BaseEnum):
    TOAST = "toast"  # 仅弹窗


class UIType(BaseEnum):
    """ "UI 组件的类型"""

    STATUS = "state"  # 状态显示
    CLICK = "click"  # 界面上的按钮
    CONFIRM = "confirm"  # 弹窗的确认按钮
    LOGBOX = "logbox"  # 日志框
    JOYSTICK = "joystick"  # 遥杆


_config: Dict[UIType, Dict[str, Type["BaseUIConfig"]]] = {}


@dataclass
class BaseUIConfig:
    type: UIType = field()
    id: str = field(default="base")
    name: str = field(default="")
    _skip = False  # 是否在创建config时收集(比如一些父类需要避免收集)

    def __init_subclass__(cls):
        if cls.skip():
            return
        default_config = _config.get(cls.type, {})
        id = cls.id if cls.id is not None else get_classname(cls)
        if id in default_config:
            raise KeyError(f"UI id: {id} is exists!")
        default_config[id] = cls
        _config[cls.type] = default_config

    @classmethod
    def to_dict(cls):
        out = {}
        for filed in fields(cls):
            if filed.name.startswith("_"):
                continue
            value = getattr(cls, filed.name)
            if hasattr(value, "value"):
                value = value.value
            out[filed.name] = value
        return out
    
    
    @classmethod
    def skip(cls):
        """当某个类直接定义skip且为True, 则跳过更新"""
        return "_skip" in cls.__dict__ and cls._skip

    @classmethod
    def create_config(cls):
        return {
            ui_type.value: {key: config.to_dict() for key, config in ui_configs.items()}
            for ui_type, ui_configs in _config.items()
        }

        return {
            "state": {
                "basic_state_desc": {
                    "name": "设备状态",
                    "default": "未知",
                    "order": 0,
                },
                "motion_mode": {
                    "name": "运动模式",
                    "default": "未知",
                    "order": 0,
                },
                "gait_desc": {"name": "机器人步态", "default": "未知"},
                "max_forward_vel": {
                    "name": "最大前进速度",
                    "default": "未知",
                },
                "position_desc": {"name": "位姿信息", "default": "未知"},
                "position_desc1": {"name": "位姿信息", "default": "未知"},
                "position_desc2": {"name": "位姿信息", "default": "未知"},
                "position_desc3": {"name": "位姿信息", "default": "未知"},
                "position_desc4": {"name": "位姿信息", "default": "未知"},
                "position_desc5": {"name": "位姿信息", "default": "未知"},
                "position_desc6": {"name": "位姿信息", "default": "未知"},
                "position_desc7": {"name": "位姿信息", "default": "未知"},
                "position_desc8": {"name": "位姿信息", "default": "未知"},
                "position_desc9": {"name": "位姿信息", "default": "未知"},
                "position_desc10": {"name": "位姿信息", "default": "未知"},
                "position_desc11": {"name": "位姿信息", "default": "未知"},
                "position_desc12": {"name": "位姿信息", "default": "未知"},
                "position_desc13": {"name": "位姿信息", "default": "未知"},
                "position_desc14": {"name": "位姿信息", "default": "未知"},
                "position_desc15": {"name": "位姿信息", "default": "未知"},
                "position_desc16": {"name": "位姿信息", "default": "未知"},
            },
            "click": {
                "emergency_stop": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "toggle-stand-down": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/pose/toggle-stand-down",
                    "tip": "切换站立/蹲下",
                    "name": "站立/蹲下",
                },
                "emergency_stop2": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop3": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop4": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop5": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop6": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop7": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop8": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop9": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop10": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop11": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop12": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop13": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop14": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
                "emergency_stop15": {
                    "type": "toast",
                    "method": "POST",
                    "url": "/emergecy_stop",
                    "tip": "紧急停止设备",
                    "name": "急停",
                },
            },
            "joystick": {"速度控制": {"method": "POST", "url": "/move/joystick"}},
            "confirm": {},
            "logbox": [
                {
                    "type": "info",
                    "label": "信息",
                    "style": {
                        "color": "#0958D9",
                        "backgroundColor": "#EBF4FF",
                        "borderColor": "#B3D1FF",
                    },
                },
                {
                    "type": "error",
                    "label": "错误",
                    "style": {
                        "color": "#D83030",
                        "backgroundColor": "#FFEDED",
                        "borderColor": "#FFB3B3",
                    },
                },
                {
                    "type": "warn",
                    "label": "警告",
                    "style": {
                        "color": "#FF6D00",
                        "backgroundColor": "#FFF2E8",
                        "borderColor": "#FFC080",
                    },
                },
                {
                    "type": "debug",
                    "label": "调试",
                    "style": {
                        "color": "#6B7280",
                        "backgroundColor": "#F3F4F6",
                        "borderColor": "#D1D5DB",
                    },
                },
            ],
        }


@dataclass
class ButtonConfig(BaseUIConfig):
    _skip = True  # 跳过收集
    type = UIType.CLICK
    button_type: ButtonType = field(default=None)
    order: int = field(default=9999)  # 显示的顺序, 值越大越靠后


@dataclass
class StatusConfig(BaseUIConfig):
    _skip = True  # 跳过收集
    type = UIType.STATUS
    default: str = field(default="未知")  # 默认显示的值
    order: int = field(default=9999)  # 显示的顺序, 值越大越靠后
