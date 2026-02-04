import { Card } from "antd";
import { useAppContext } from "../context/AppContext";
import { sortByOrder } from "../utils";

export interface StateItemConfig {
  name: string; // 显示文字
  default: string; // 默认值
  order?: number; // 排序，越小越靠前，默认999
}

export const StateDisplay = () => {
  const { stateData, config } = useAppContext();
  // 按order排序状态项，几十个状态也能有序展示
  const sortedState = sortByOrder(Object.entries(config?.state || []));

  return (
    <Card
      title="设备状态展示"
      className="mb-6 w-full"
      variant="borderless"
      style={{ boxShadow: "0 2px 8px rgba(0,0,0,0.08)" }}
    >
      {/* 核心：全端Grid布局，每行若干列（响应式适配）
          xs:3列（手机竖屏，核心需求：每行3个）
          sm:4列（小平板/大屏手机）
          md:4列（平板）
          lg:6列（小屏PC）
          xl:8列（大屏PC，充分利用空间）
          gap：响应式间距，移动端更小更紧凑
      */}
      <div
        className="grid grid-cols-3 sm:grid-cols-3 md:grid-cols-4 lg:grid-cols-6 xl:grid-cols-8 
                      gap-3 sm:gap-4 md:gap-4 lg:gap-4 w-full"
      >
        {sortedState.map(([key, item]) => (
          <div
            key={key}
            className="rounded-lg shadow-sm bg-slate-50
                      p-2 sm:p-3 md:p-3 lg:p-3 // 响应式内边距，移动端更紧凑
                      h-full flex flex-col justify-start"
          >
            {/* 状态名称：移动端更小字体，超出自动截断 */}
            <div className="text-xs sm:text-sm text-gray-500 mb-1 truncate">
              {item.name}
            </div>
            {/* 状态值：移动端稍小字体，粗体，长数值自动换行 */}
            <div className="text-base sm:text-lg font-bold text-gray-800 break-all">
              {stateData[key] as string}
            </div>
          </div>
        ))}
      </div>
    </Card>
  );
};
