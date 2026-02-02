import json
from dataclasses import dataclass, asdict, field, fields
from enum import Enum
from typing import Dict, List, Any, Optional, TypeVar

from event_callback.core import R
from event_callback.utils import get_classname

T = TypeVar("T")


class ButtonEventType(Enum):
    """按钮触发事件类型枚举"""

    TOAST = "toast"
    MODAL = "modal"
    COPY = "copy"


class LogboxDataType(Enum):
    """日志框数据类型枚举"""

    ERROR = "error"
    INFO = "info"
    WARN = "warn"
    DEBUG = "debug"


class FormItemType(Enum):
    """表单控件类型枚举"""

    SLIDER = "slider"
    INPUT = "input"
    NUMBER = "number"
    SELECT = "select"
    CHECKBOX = "checkbox"
    SWITCH = "switch"


@dataclass
class BaseUIConfig:
    """所有组件配置的基类，提供通用序列化和更新能力"""

    config_id: str = field(default=None)  # 组件唯一标识
    _id_cnt = 0

    def __post_init__(self):
        if not hasattr(self, "config_type"):
            raise NotImplementedError("子类必须定义config_type类变量")

        # 处理config_id的自动生成和唯一性检查
        if self.config_id is None:
            self.config_id = self.generate_id()

        config_map = R._config_store.get(self.config_type, {})
        if self.config_id in config_map:
            raise ValueError(f"{self.config_type}组件中已存在ID：{self.config_id}")
        config_map[self.config_id] = self
        R._config_store[self.config_type] = config_map

    @classmethod
    def generate_id(cls):
        _id = str(get_classname(cls, is_method=False)) + str(cls._id_cnt)
        cls._id_cnt += 1
        return _id

    def to_dict(self) -> Dict[str, Any]:
        """
        转换为字典（过滤None值，枚举转字符串）
        :return: 格式化的配置字典
        """

        def _convert(value):
            if isinstance(value, Enum):
                return value.value
            elif isinstance(value, (list, tuple)):
                return [_convert(item) for item in value]
            elif isinstance(value, dict):
                return {k: _convert(v) for k, v in value.items()}
            return value

        raw_dict = asdict(self)
        return {k: _convert(v) for k, v in raw_dict.items() if v is not None}

    def update(self, **kwargs) -> None:
        """
        动态更新配置项（支持字符串自动转枚举）
        :param kwargs: 待更新的配置项键值对
        :raise ValueError: 配置项不存在或枚举值无效
        """
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise ValueError(f"组件{self.id}不支持配置项：{key}")

            # 处理枚举类型：字符串自动转换为枚举实例
            field_type = next(f for f in fields(self) if f.name == key)
            if isinstance(field_type, type) and issubclass(field_type, Enum):
                if isinstance(value, str):
                    try:
                        value = field_type(value)
                    except ValueError:
                        valid_values = [e.value for e in field_type]
                        raise ValueError(
                            f"配置项{key}有效值：{valid_values}，传入：{value}"
                        )

            setattr(self, key, value)

    @staticmethod
    def generate_json() -> Dict[str, Any]:
        """
        生成JSON格式的前端配置
        :return: JSON对象
        """

        def serialize(obj):
            """递归序列化：处理枚举、列表、字典和配置实例"""
            if isinstance(obj, Enum):
                return obj.value
            elif isinstance(obj, (list, tuple)):
                return [serialize(item) for item in obj]
            elif isinstance(obj, dict):
                return {k: serialize(v) for k, v in obj.items()}
            elif hasattr(obj, "to_dict"):
                return obj.to_dict()
            return obj

        json_data = {}
        for component_type, configs in R._config_store.items():
            json_data[component_type] = {
                config_id: serialize(config) for config_id, config in configs.items()
            }
        return json_data


@dataclass
class BaseFormItemConfig:
    """表单控件基类"""

    name: str  # 控件显示名称
    type: FormItemType  # 控件类型（枚举）

    def to_dict(self) -> Dict[str, Any]:
        """序列化表单控件配置"""
        raw_dict = asdict(self)
        return {
            k: (v.value if isinstance(v, Enum) else v)
            for k, v in raw_dict.items()
            if v is not None
        }


@dataclass
class SwitchFormItemConfig(BaseFormItemConfig):
    """开关控件配置"""

    type: FormItemType = field(default=FormItemType.SWITCH, init=False)
    default: Optional[bool] = None


@dataclass
class SliderFormItemConfig(BaseFormItemConfig):
    """滑块控件配置"""

    type: FormItemType = field(default=FormItemType.SLIDER, init=False)
    max: float
    min: float
    step: float = 1.0
    default: Optional[float] = None


@dataclass
class InputFormItemConfig(BaseFormItemConfig):
    """输入框控件配置"""

    type: FormItemType = field(default=FormItemType.INPUT, init=False)
    placeholder: Optional[str] = None
    maxlength: Optional[int] = None
    default: Optional[str] = None


@dataclass
class NumberFormItemConfig(BaseFormItemConfig):
    """数字输入框控件配置"""

    type: FormItemType = field(default=FormItemType.NUMBER, init=False)
    min: Optional[int] = None
    max: Optional[int] = None
    default: Optional[int] = None
    precision: Optional[int] = None


@dataclass
class SelectFormItemConfig(BaseFormItemConfig):
    """下拉框控件配置"""

    type: FormItemType = field(default=FormItemType.SELECT, init=False)
    options: List[Dict[str, str]]
    multiple: bool = False
    default: Optional[str | List[str]] = None


@dataclass
class CheckboxFormItemConfig(BaseFormItemConfig):
    """复选框控件配置"""

    type: FormItemType = field(default=FormItemType.CHECKBOX, init=False)
    checked: bool = False
    label_position: str = "left"
    default: Optional[bool] = None


# -------------------------- 业务组件配置类 --------------------------
@dataclass
class StatusConfig(BaseUIConfig):
    """状态栏配置"""

    config_type: str = "state"
    name: str = None  # 显示名称
    default: str = "未知"


@dataclass
class ButtonConfig(BaseUIConfig):
    """按钮配置"""

    config_type: str = "button"
    name: str = None  # 显示文字
    order: int = 9999  # 显示顺序
    button_type: ButtonEventType = None  # 触发事件类型
    url: Optional[str] = None  # 请求地址（可选）
    method: Optional[str] = None  # 请求方法（可选）
    form_id: Optional[str] = None


@dataclass
class FormConfig(BaseUIConfig):
    """表单配置"""

    config_type: str = "form"
    target_id: str = None  # 按钮的id
    url: Optional[str] = None  # 提交地址（可选）
    method: Optional[str] = None  # 提交方法（可选）
    items: List[BaseFormItemConfig] = field(default_factory=list)  # 表单项列表


@dataclass
class LogboxConfig(BaseUIConfig):
    """日志框配置"""

    config_type: str = "logbox"
    type: LogboxDataType = None  # 日志类型
    label: str = None  # 显示标签
    color: str = None  # 文字颜色
    backgroundColor: str = None  # 背景色
    borderColor: str = None  # 边框色


LogboxConfig(
    type=LogboxDataType.ERROR,
    color="#D83030",
    backgroundColor="#FFEDED",
    borderColor="#FFB3B3",
    label="error",
)

LogboxConfig(
    type=LogboxDataType.INFO,
    color="#0958D9",
    backgroundColor="#EBF4FF",
    borderColor="#B3D1FF",
    label="info",
)

LogboxConfig(
    type=LogboxDataType.WARN,
    color="#FF6D00",
    backgroundColor="#FFF2E8",
    borderColor="#FFC080",
    label="warn",
)

LogboxConfig(
    type=LogboxDataType.DEBUG,
    color="#6B7280",
    backgroundColor="#F3F4F6",
    borderColor="#D1D5DB",
    label="debug",
)


@dataclass
class JoystickConfig(BaseUIConfig):
    """虚拟摇杆配置"""

    config_type: str = "joystick"
    url: Optional[str] = None  # 控制请求地址（可选）
    method: Optional[str] = None  # 控制请求方法（可选）


@dataclass
class APIConfig:
    button_config: BaseUIConfig = field(default=None)
    form_config: FormConfig = field(default=None)

    def update(self, url: str, method: str):
        if self.button_config is not None:
            self.button_config.update(url=url, method=method)
        if self.form_config is not None:
            self.form_config.update(url=url, method=method)
