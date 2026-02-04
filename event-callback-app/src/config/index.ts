import type { ButtonItemConfig } from "../components/ButtonGroup";
import type { StateItemConfig } from "../components/StateDisplay";

// 确认按钮配置接口
export interface ConfirmItemConfig {
  target: string; // 关联的click按钮key
  method: "GET" | "POST" | "PUT" | "DELETE";
  url: string;
}

export interface JoystickItemConfig {
  method: "GET" | "POST" | "PUT" | "DELETE";
  url: string;
}

// 全局配置接口
export interface GlobalConfig {
  state: Record<string, StateItemConfig>; // 状态展示配置
  button: Record<string, ButtonItemConfig>; // 控制按钮配置
  joystick: Record<string, JoystickItemConfig>; //虚拟摇杆操作配置
  logbox: Record<string, LogTypeConfig>; // 日志框的配置
}

// 核心类型定义（完全不变）
export interface LogTypeConfig {
  type: string;
  label: string;
  color: string;
  borderColor: string;
  backgroundColor: string;
}
