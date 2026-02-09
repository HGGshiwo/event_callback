import React, { useState, useMemo, type ReactNode, type Ref } from "react";
import { Table, Tree, Input, Typography } from "antd";
import type { TableProps, TreeProps } from "antd";
import type { InputRef } from "antd/es/input/Input";
import type { DataNode } from "antd/es/tree";
import type { FormConfig, FormItemConfig } from "./ModalForm";

// 原始参数类型（保留核心字段，支持任意列扩展）
export type ParamItem = {
  id: string; // 唯一标识 必传
  group: string; // 分组（支持多级，如 "A_B_C"）必传
  [key: string]: string | number | string[]; // 扩展列（name/help/default/value等）
};

// 列配置项类型（自定义列展示、编辑权限、宽度）
export type ColumnConfig = {
  key: string; // 对应ParamItem的字段名 必传
  title: string; // 表格列标题 必传
  editable: boolean; // 是否可编辑 必传
  width?: string | number; // 可选列宽度
  ellipsis?: boolean; // 可选是否超出省略
};

// 组件核心Props类型
export type GroupTableProps = {
  /** 数据源（受控属性，与Form联动的核心） */
  value: ParamItem[];
  /** 编辑回调：(唯一id, 修改的列名, 列新值) => void，适配Form.Item联动 */
  onChange: (id: string, colName: string, value: string | number) => void;
  /** 自定义列配置，不传则使用默认配置 */
  columnsConfig?: ColumnConfig[];
  /** 搜索框自定义占位符 */
  searchPlaceholder?: string;
  /** 树形侧边栏默认展开的节点key（与group的多级路径一致） */
  defaultExpandedKeys?: string[];
  /** 组件整体高度，默认600px */
  height?: string | number;
  /** 树形侧边栏宽度，默认240px */
  sidebarWidth?: string | number;
};

// 默认列配置（与原组件的名称/说明/默认值/当前值一一对应）
const DEFAULT_COLUMNS: ColumnConfig[] = [
  { key: "name", title: "名称", editable: false, width: 150, ellipsis: true },
  { key: "help", title: "说明", editable: false, width: 220, ellipsis: true },
  { key: "default", title: "默认值", editable: false, width: 120 },
  { key: "value", title: "当前值", editable: true, width: 120 },
];

// 构建antd Tree所需的树形节点（适配group的下划线多级拆分）
const buildTreeData = (items: ParamItem[]): DataNode[] => {
  const root: DataNode[] = [{ title: "全部", key: "__ALL__" }]; // 固定根节点-全部
  const nodeMap = new Map<string, DataNode>(); // 缓存节点，避免重复创建

  items.forEach((item) => {
    if (!item.group) return; // 无分组的项不参与树形构建
    const groupParts = item.group.split("_");
    let parentKey = "";
    let currentPath = "";

    // 逐层拆分group，构建多级树形节点
    groupParts.forEach((part, index) => {
      currentPath = index === 0 ? part : `${currentPath}_${part}`;
      if (!nodeMap.has(currentPath)) {
        const node: DataNode = {
          title: part,
          key: currentPath,
          //   parentKey: parentKey || undefined,
        };
        nodeMap.set(currentPath, node);
        // 挂载到父节点/根节点
        if (parentKey) {
          const parentNode = nodeMap.get(parentKey);
          if (parentNode) {
            parentNode.children = parentNode.children || [];
            parentNode.children.push(node);
          }
        } else {
          root.push(node);
        }
      }
      parentKey = currentPath;
    });
  });

  return root;
};

// 可编辑单元格子组件（抽离复用，适配antd Table的render）
type OnChangeFunc = (val: CellValue) => void;
type CellValue = string | number;
const EditableCell: React.FC<{
  value: CellValue;
  renderChildren: (
    value: string,
    onChange: OnChangeFunc,
    ref: Ref<any>,
  ) => ReactNode;
  onChange: OnChangeFunc;
}> = ({ value, onChange, renderChildren }) => {
  const [editing, setEditing] = useState(false);
  const [inputValue, setInputValue] = useState(value);
  const inputRef = React.useRef<InputRef>(null);

  // 编辑状态自动聚焦输入框
  React.useEffect(() => {
    if (editing) {
      inputRef.current?.focus();
    }
  }, [editing]);

  // 同步外部传入的value（受控适配）
  React.useEffect(() => {
    setInputValue(value);
  }, [value]);

  // 结束编辑：失焦/回车触发，Esc仅关闭编辑不触发回调
  const handleFinish = (triggerByEsc = false) => {
    setEditing(false);
    if (!triggerByEsc) {
      onChange(inputValue);
    }
  };

  if (editing) {
    return renderChildren(
      inputValue,
      (e) => setInputValue(e.target.value),
      inputRef,
    );
  }

  return (
    <Typography.Text
      onClick={() => setEditing(true)}
      style={{ cursor: "pointer", color: "#1677ff", userSelect: "none" }}
    >
      {value ?? "-"}
    </Typography.Text>
  );
};

// 核心组件实现（无Form包裹，纯受控组件，适配Form.Item）
export const GroupTable: React.FC<GroupTableProps> = ({
  value = [],
  onChange,
  columnsConfig = DEFAULT_COLUMNS,
  searchPlaceholder = "搜索分组/名称/说明",
  defaultExpandedKeys = [],
  height = 600,
  sidebarWidth = 240,
}) => {
  // 组件内部状态：搜索关键词、树形选中/展开节点
  const [searchKeyword, setSearchKeyword] = useState("");
  const [selectedKeys, setSelectedKeys] = useState<string[]>(["__ALL__"]);
  const [expandedKeys, setExpandedKeys] =
    useState<string[]>(defaultExpandedKeys);

  // 1. 搜索过滤：根据关键词过滤数据源（useMemo避免重复计算）
  const filteredBySearch = useMemo(() => {
    if (!searchKeyword.trim()) return value;
    const keyword = searchKeyword.trim().toLowerCase();
    return value.filter((item) => {
      return (
        (item.group && item.group.toLowerCase().includes(keyword)) ||
        (item.name && (item.name as string).toLowerCase().includes(keyword)) ||
        (item.help && (item.help as string).toLowerCase().includes(keyword))
      );
    });
  }, [value, searchKeyword]);

  // 2. 树形选中过滤：结合搜索结果，过滤表格最终展示数据
  const tableData = useMemo(() => {
    if (selectedKeys[0] === "__ALL__") return filteredBySearch;
    const selectedGroup = selectedKeys[0];
    return filteredBySearch.filter(
      (item) => item.group && item.group.startsWith(selectedGroup),
    );
  }, [filteredBySearch, selectedKeys]);

  // 3. 构建表格列：根据列配置自动生成，区分可编辑/不可编辑列
  const tableColumns = useMemo(() => {
    return columnsConfig.map((config) => ({
      title: config.title,
      dataIndex: config.key,
      key: config.key,
      width: config.width,
      ellipsis: config.ellipsis ?? false,
      render: (cellValue: string | number, record: ParamItem) => {
        // 可编辑列渲染EditableCell，不可编辑列直接展示（处理空值/数组）
        if (config.editable) {
          return (
            <EditableCell
              value={cellValue ?? ""}
              onChange={(newValue) => onChange(record.id, config.key, newValue)}
            />
          );
        }
        // 处理原组件的_help数组类型，以及普通空值
        return Array.isArray(cellValue)
          ? cellValue.join("\n")
          : (cellValue ?? "-");
      },
    }));
  }, [columnsConfig, onChange]);

  // 4. 树形组件配置（抽离props，代码更整洁）
  const treeProps: TreeProps = {
    treeData: buildTreeData(filteredBySearch),
    selectedKeys,
    expandedKeys,
    onSelect: (keys) => setSelectedKeys(keys),
    onExpand: (keys) => setExpandedKeys(keys),
    showLine: true, // 显示树形连接线
    defaultExpandAll: false,
    style: {
      width: "100%",
      overflowY: "auto",
      flex: 1,
    },
  };

  // 5. 表格组件配置（适配高度、分页、空状态）
  const tableProps: TableProps<ParamItem> = {
    dataSource: tableData,
    columns: tableColumns,
    rowKey: "id", // 必传，基于唯一id做行标识
    pagination: {
      pageSize: 10,
      showSizeChanger: true,
      showTotal: (total) => `共 ${total} 条数据`,
    },
    scroll: { y: "calc(100% - 40px)" }, // 自适应高度，预留搜索框空间
    style: { width: "100%", overflow: "auto" },
    locale: { emptyText: "暂无匹配数据" }, // 空状态提示
    size: "middle",
  };

  // 组件整体布局：左侧树形侧边栏 + 右侧表格区域（弹性布局，高度自适应）
  return (
    <div
      style={{
        display: "flex",
        gap: 16,
        padding: 8,
        height: height,
        boxSizing: "border-box",
      }}
    >
      {/* 左侧：搜索框 + 树形栏 */}
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          gap: 8,
          width: sidebarWidth,
        }}
      >
        <Input
          placeholder={searchPlaceholder}
          value={searchKeyword}
          onChange={(e) => {
            setSearchKeyword(e.target.value);
            setSelectedKeys(["__ALL__"]); // 搜索时重置树形选中为「全部」
          }}
          allowClear // 带清除按钮
          size="small"
        />
        <div
          style={{ display: "flex", flexDirection: "column", height: "100%" }}
        >
          <Tree {...treeProps} />
        </div>
      </div>

      {/* 右侧：表格区域（占满剩余宽度） */}
      <div
        style={{
          display: "flex",
          flexDirection: "column",
          flex: 1,
          height: "100%",
        }}
      >
        <Table {...tableProps} />
      </div>
    </div>
  );
};
