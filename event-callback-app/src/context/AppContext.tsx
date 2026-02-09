import {
  createContext,
  useContext,
  type ReactNode,
  useState,
  useEffect,
  useRef,
} from "react";
import { type GlobalConfig } from "../config";
import { httpRequest, sortByOrder, wsURL } from "../utils";
import { Spin } from "antd";

export interface LogItemData {
  timestamp: number;
  content: string;
}

// 上下文状态类型
interface AppContextState {
  stateData: Record<string, LogItemData[] | string>; // 配置化状态数据（与config.state对应）
  wsStatus: "connecting" | "open" | "closed" | "error"; // WS连接状态
  modalVisible: boolean; // Modal显隐
  modalKey: string; // 当前打开的Modal对应的click key
  modalFormData: Record<string, unknown>; // Modal表单数据
  config: GlobalConfig | null; // 全局配置
  setModalVisible: (visible: boolean, key?: string) => void; // 设置Modal显隐
  updateModalFormData: (data: Record<string, unknown> | object) => void; // 更新Modal表单数据
  resetModalFormData: () => void; // 重置Modal表单数据
}

// 创建上下文，默认值为空（后续用Provider包裹）
const AppContext = createContext<AppContextState>({} as AppContextState);

// 上下文Provider组件，包裹根组件
export const AppProvider = ({ children }: { children: ReactNode }) => {
  // 状态定义
  const [stateData, setStateData] = useState<
    Record<string, string | LogItemData[]>
  >({});
  const [wsStatus, setWsStatus] =
    useState<AppContextState["wsStatus"]>("connecting");
  const [config, setConfig] = useState<GlobalConfig | null>(null);

  const [modalVisible, setModalVisible] = useState(false);
  const [modalKey, setModalKey] = useState("");
  const [modalFormData, setModalFormData] = useState<Record<string, unknown>>(
    {},
  );

  // 保存WebSocket实例（持久化，不随组件重渲染丢失）
  const wsInstance = useRef<WebSocket | null>(null);
  // 重连定时器引用（用于清理重复的重连任务）
  const reconnectTimer = useRef<number | null>(null);
  // 标记是否手动关闭连接（避免手动关闭后自动重连）
  const isManualClose = useRef(false);

  // 初始化/重建WebSocket连接
  const initWebSocket = () => {
    // 先关闭已有连接
    if (wsInstance.current) {
      wsInstance.current.close();
      wsInstance.current = null;
    }

    // 标记为非手动关闭，允许重连
    isManualClose.current = false;
    // 更新连接状态为正在连接
    setWsStatus("connecting");

    try {
      // 创建原生WebSocket实例
      const ws = new WebSocket(wsURL);

      // 连接成功回调
      ws.onopen = () => {
        console.log("WebSocket连接成功");
        setWsStatus("open");
        // 清除重连定时器（连接成功后无需重连）
        if (reconnectTimer.current) {
          clearTimeout(reconnectTimer.current);
          reconnectTimer.current = null;
        }
      };

      // 连接关闭回调
      ws.onclose = (event) => {
        console.log("WebSocket连接关闭", event);
        setWsStatus("closed");
        // 非手动关闭时触发重连
        if (!isManualClose.current) {
          scheduleReconnect();
        }
      };

      // 连接错误回调
      ws.onerror = (error) => {
        console.error("WebSocket连接错误", error);
        setWsStatus("error");
        // 错误时触发重连
        if (!isManualClose.current) {
          scheduleReconnect();
        }
      };

      // 接收消息回调（核心：直接处理消息，不依赖State中转）
      ws.onmessage = (event) => {
        try {
          // 直接解析消息数据，避免通过lastMessage State中转
          const data = JSON.parse(event.data);
          // 忽略不符合格式的数据
          if (!data.type) return;

          const dataType = data.type;
          delete data.type;

          // 根据消息类型更新状态数据
          setStateData((prev) => {
            if (dataType === "state") {
              // state类型：合并更新状态数据
              return { ...prev, ...data };
            } else {
              // 其他类型：追加到数组（如日志类数据）
              const currentList = (prev[dataType] || []) as LogItemData[];
              return { ...prev, [dataType]: [...currentList, data] };
            }
          });
        } catch (error) {
          console.error("WS消息解析失败：", error);
        }
      };

      // 保存WebSocket实例到ref
      wsInstance.current = ws;
    } catch (error) {
      console.error("创建WebSocket实例失败", error);
      setWsStatus("error");
      scheduleReconnect();
    }
  };

  // 调度重连（带间隔，避免频繁重连）
  const scheduleReconnect = () => {
    // 清除已有定时器，避免重复
    if (reconnectTimer.current) {
      clearTimeout(reconnectTimer.current);
    }
    // 3秒后重连（与原逻辑保持一致）
    reconnectTimer.current = setTimeout(() => {
      console.log("尝试重新连接WebSocket...");
      initWebSocket();
    }, 3000);
  };

  // 初始化配置并创建WebSocket连接
  useEffect(() => {
    (async () => {
      try {
        const res = await httpRequest("GET", "/page_config");
        const config = (res || null) as GlobalConfig;
        setConfig(config);

        // 初始化状态数据（按config.state的default值，且按order排序）
        if (config?.state) {
          const stateEntries = sortByOrder(Object.entries(config.state));
          const initStateData = stateEntries.reduce(
            (prev, [key, item]) => {
              prev[key] = item.default;
              return prev;
            },
            {} as Record<string, string>,
          );
          setStateData(initStateData);
        }

        // 配置加载完成后初始化WebSocket连接
        initWebSocket();
      } catch (error) {
        console.error("加载页面配置失败", error);
      }
    })();

    // 组件卸载时清理WebSocket和定时器
    return () => {
      // 标记为手动关闭
      isManualClose.current = true;
      // 清除重连定时器
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
      }
      // 关闭WebSocket连接
      if (wsInstance.current) {
        wsInstance.current.close();
      }
    };
  }, []);

  // 设置Modal显隐（同时设置关联的key）
  const handleSetModalVisible = (visible: boolean, key?: string) => {
    setModalVisible(visible);
    if (key) setModalKey(key);
    if (!visible) resetModalFormData(); // 关闭时重置表单
  };

  // 更新Modal表单数据
  const updateModalFormData = (data: Record<string, unknown> | object) => {
    setModalFormData((prev) => ({ ...prev, ...data }));
  };

  // 重置Modal表单数据
  const resetModalFormData = () => {
    setModalFormData({});
    setModalKey("");
  };

  // 上下文值
  const contextValue: AppContextState = {
    stateData,
    wsStatus,
    modalVisible,
    modalKey,
    modalFormData,
    config,
    setModalVisible: handleSetModalVisible,
    updateModalFormData,
    resetModalFormData,
  };

  return (
    <Spin
      spinning={wsStatus !== "open"}
      delay={500}
      size="large"
      tip="尝试websocket连接中..."
    >
      <AppContext.Provider value={contextValue}>{children}</AppContext.Provider>
    </Spin>
  );
};

// 自定义Hook，简化上下文使用
export const useAppContext = () => useContext(AppContext);
