import {
  createContext,
  useContext,
  type ReactNode,
  useState,
  useEffect,
} from "react";
import useWebSocket from "react-use-websocket";
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
  const [stateData, setStateData] =
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    useState<Record<string, string | any[]>>({});
  const [wsStatus, setWsStatus] =
    useState<AppContextState["wsStatus"]>("connecting");
  const [curWsURL, setCurWsURL] = useState<string | null>(null); // 当获取到配置的时候再进行ws连接
  const [config, setConfig] = useState<GlobalConfig | null>(null);

  const [modalVisible, setModalVisible] = useState(false);
  const [modalKey, setModalKey] = useState("");
  const [modalFormData, setModalFormData] = useState<Record<string, unknown>>(
    {},
  );

  const { lastMessage, readyState } = useWebSocket(curWsURL, {
    retryOnError: true, // 错误时重连
    reconnectAttempts: -1, // 无限重连（-1），可设具体次数
    reconnectInterval: 3000, // 重连间隔3秒
    onOpen: () => setWsStatus("open"),
    onClose: () => setWsStatus("closed"),
    onError: () => setWsStatus("error"),
  });

  useEffect(() => {
    (async () => {
      const res = await httpRequest("GET", "/page_config");
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      const config = ((res as any) || null) as GlobalConfig;
      setConfig(config);

      // 初始化状态数据（按config.state的default值，且按order排序）
      const initStateData = () => {
        const stateEntries = sortByOrder(Object.entries(config.state || {}));
        return stateEntries.reduce(
          (prev, [key, item]) => {
            prev[key] = item.default;
            return prev;
          },
          {} as Record<string, string>,
        );
      };
      setStateData(initStateData);
      // WebSocket连接（/ws地址，开启自动重连）
      setCurWsURL(wsURL);
    })();
  }, []);

  // 监听WS连接状态变化
  useEffect(() => {
    switch (readyState) {
      case 0:
        // eslint-disable-next-line react-hooks/set-state-in-effect
        setWsStatus("connecting");
        break;
      case 1:
        setWsStatus("open");
        break;
      case 2:
        setWsStatus("closed");
        break;
      case 3:
        setWsStatus("error");
        break;
      default:
        setWsStatus("closed");
    }
  }, [readyState]);

  // 监听WS消息，更新stateData（解析message中的state字段）
  useEffect(() => {
    if (lastMessage && wsStatus === "open") {
      try {
        const data = JSON.parse(lastMessage.data);
        // 仅处理state字段，且字段在config.state中存在时更新
        if (!data.type) return; // 忽略不符合格式的数据
        const data_type = data.type;
        delete data.type;
        if (data_type == "state") {
          // eslint-disable-next-line react-hooks/set-state-in-effect
          setStateData((prev) => ({ ...prev, ...data }));
        } else {
          setStateData((prev) => {
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            let temp_data = (prev[data_type] || []) as any[];
            temp_data = [...temp_data, data];
            return { ...prev, [data_type]: temp_data };
          });
        }
      } catch (error) {
        console.error("WS消息解析失败：", error);
      }
    }
  }, [lastMessage, wsStatus]);

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
      spinning={wsStatus != "open"}
      delay={500}
      size="large"
      tip="尝试websocket连接中..."
    >
      <AppContext.Provider value={contextValue}>{children}</AppContext.Provider>
    </Spin>
  );
};

// 自定义Hook，简化上下文使用
// eslint-disable-next-line react-refresh/only-export-components
export const useAppContext = () => useContext(AppContext);
