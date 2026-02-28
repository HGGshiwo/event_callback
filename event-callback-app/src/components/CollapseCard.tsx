import { useState } from "react";
import { sortByOrder } from "../utils";
import { Button, Card, Modal, type CardProps } from "antd";

interface CardItemConfig {
  collapse?: boolean;
  order?: number;
}

export interface CollapseCardProps extends Omit<
  CardProps,
  "config" | "renderItem"
> {
  config: Record<string, CardItemConfig>;
  title: string;
  renderItem: (key: string, config: CardItemConfig) => React.ReactNode;
}

export const CollapseCard: React.FC<CollapseCardProps> = ({
  config,
  title,
  renderItem,
  ...rest
}) => {
  const [modalOpen, setModalOpen] = useState(false);

  const sortedConfig = sortByOrder(Object.entries(config));
  const displayKey = sortedConfig
    .filter(([, item]) => item.collapse === false)
    .map(([key]) => key);

  return (
    <>
      <Card
        title={title}
        className="mb-2 w-full"
        variant="borderless"
        style={{ boxShadow: "0 2px 8px rgba(0,0,0,0.08)" }}
        extra={
          <Button type="text" onClick={() => setModalOpen(true)}>
            展开更多
          </Button>
        }
        {...rest}
      >
        <div className="grid grid-cols-3 sm:grid-cols-4 md:grid-cols-6 lg:grid-cols-8 xl:grid-cols-8 gap-3  sm:gap-4 md:gap-4 lg:gap-4 w-full">
          {displayKey.map((key) => {
            return renderItem(key, config[key]);
          })}
        </div>
      </Card>
      <Modal
        title={`${title} - 完整数据`}
        open={modalOpen}
        onCancel={() => setModalOpen(false)}
        footer={null}
        width={800}
      >
        <div className="grid grid-cols-3 sm:grid-cols-4 md:grid-cols-6 gap-3 sm:gap-4 md:gap-4 lg:gap-4 w-full">
          {sortedConfig.map(([key, configItem]) => {
            return renderItem(key, configItem);
          })}
        </div>
      </Modal>
    </>
  );
};
