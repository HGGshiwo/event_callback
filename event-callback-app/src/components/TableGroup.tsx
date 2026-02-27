/* eslint-disable @typescript-eslint/no-explicit-any */
import { Card } from "antd";
import { useAppContext } from "../context/AppContext";
import Table, { type TableColumnConfig } from "./Table";
import { useMemo } from "react";

export interface TableGroupConfig {
  columns: Record<string, TableColumnConfig>;
}

export default function TableGroup() {
  const { config, stateData } = useAppContext();
  const tableConfig = useMemo(() => config?.table || {}, [config?.table]);

  const hasData = useMemo(() => {
    return Object.keys(tableConfig).some((key) => !!stateData[key]?.length);
  }, [stateData, tableConfig]);

  return (
    hasData && (
      <Card
        title="设备状态展示2"
        className="mb-6 w-full"
        variant="borderless"
        style={{ boxShadow: "0 2px 8px rgba(0,0,0,0.08)" }}
      >
        {Object.entries(tableConfig).map(([key, item]) => {
          const columns = Object.entries(item.columns).map(
            ([colKey, item]) => ({ ...item, key: colKey }),
          );

          return (
            <Table
              key={key}
              data={(stateData[key] as any) || []}
              columns={columns}
            />
          );
        })}
      </Card>
    )
  );
}
