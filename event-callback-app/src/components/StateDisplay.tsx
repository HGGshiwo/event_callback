import { useAppContext } from "../context/AppContext";
import { CollapseCard } from "./CollapseCard";

export interface StateItemConfig {
  name: string; // 显示文字
  default: string; // 默认值
  order?: number; // 排序，越小越靠前，默认999
  collapse?: boolean; // 是否默认折叠（不展开时显示）
}

export const StateDisplay = () => {
  const { stateData, config } = useAppContext();
  const stateConfig = config?.state || {};

  return (
    <CollapseCard
      title="设备状态展示"
      className="mb-2 w-full"
      variant="borderless"
      config={stateConfig}
      renderItem={(key, cfg) => {
          const itemConfig = cfg as StateItemConfig;
          return (
            <div
              key={key}
              className="rounded-lg shadow-sm bg-slate-50
                          p-2 sm:p-3 md:p-3 lg:p-3
                          h-full flex flex-col justify-start"
            >
              <div className="text-xs sm:text-sm text-gray-500 mb-1 truncate">
                {itemConfig.name}
              </div>
              <div className="text-base sm:text-lg font-bold text-gray-800 break-all">
                {stateData[key] as string}
              </div>
            </div>
          );
        }}
    />
  );
};
