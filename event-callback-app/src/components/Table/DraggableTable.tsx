import React, {
  useState,
  useRef,
  type SetStateAction,
  type Dispatch,
  type JSX,
} from "react";
import {
  ConfigProvider,
  Table as AntTable,
  Button,
  Input,
  Checkbox,
} from "antd";
import { type InputRef, Row } from "antd";
import type { ColumnType } from "antd/lib/table";
import { AiOutlineLoading } from "react-icons/ai";
import { SearchOutlined } from "@ant-design/icons";
import type {
  TableRowSelection,
  FilterConfirmProps,
  FilterDropdownProps,
} from "antd/lib/table/interface";
import { GoChevronRight, GoChevronDown } from "react-icons/go";
import Highlighter from "react-highlight-words";
import { CSS } from "@dnd-kit/utilities";
import type { DragEndEvent } from "@dnd-kit/core";
import { DndContext } from "@dnd-kit/core";
import {
  arrayMove,
  SortableContext,
  useSortable,
  verticalListSortingStrategy,
} from "@dnd-kit/sortable";
import ReactDragListView from "react-drag-listview";
import type { DataIndex, Order, TableColumn } from "./TableColumn";

type DefaultRecordType = {
  [x: string]: any;
};

interface DraggableTableProps<T> {
  columns: TableColumn<T>[];
  dataSource: T[];
  className?: string;
  loading?: boolean;
  // Selection
  selectable?: boolean;
  onSelected?: (selectedRecords: T[]) => void;
  getCheckboxProps?: (record: T) => DefaultRecordType;
  // Expand
  expandable?: boolean;
  expandedRowRender?: (record: T) => JSX.Element;
  rowExpandable?: (record: T) => boolean;
  rowClassName?: (record: T, index?: number) => string;
  pageSize?: number;
  canChangePageSize?: boolean;
  paginationPosition?: "topRight" | "topLeft" | "bottomRight" | "bottomLeft";
  setDataSource: Dispatch<SetStateAction<T[]>>;
  setColumnsData: Dispatch<SetStateAction<TableColumn<T>[]>>;
  initialColumnsData: TableColumn<T>[];
  // 新增：拖动开关参数，默认开启
  draggable?: boolean;
}

const DraggableTable = <T extends DefaultRecordType>({
  columns,
  dataSource,
  setDataSource,
  setColumnsData,
  initialColumnsData,
  loading = false,
  selectable = false,
  onSelected = undefined,
  getCheckboxProps = undefined,
  expandable = false,
  expandedRowRender = undefined,
  rowExpandable = undefined,
  rowClassName = undefined,
  className = undefined,
  pageSize = 20,
  canChangePageSize = true,
  paginationPosition = "bottomRight",
  // 新增：解构draggable参数，默认值为true
  draggable = false,
}: DraggableTableProps<T>): JSX.Element => {
  const [searchText, setSearchText] = useState<string>("");
  const [searchedColumn, setSearchedColumn] = useState<string>("");
  const [itemsPerPage, setItemsPerPage] = useState<number>(pageSize);

  const searchInput = useRef<InputRef>(null);

  const onRowSelection = {
    onChange: onSelected
      ? (selectedRowKeys: React.Key[], selectedRows: T[]) =>
          onSelected(selectedRows)
      : undefined,
    getCheckboxProps,
  };

  let rowSelection: TableRowSelection<T> | undefined = undefined;
  if (selectable) {
    rowSelection = { type: "checkbox" };
    if (onSelected) rowSelection = { ...onRowSelection, ...rowSelection };
  }

  const handleSearch = (
    selectedKeys: string[],
    confirm: (param?: FilterConfirmProps) => void,
    dataIndex: DataIndex,
  ) => {
    confirm();
    setSearchText(selectedKeys[0]);
    setSearchedColumn(dataIndex as string);
  };

  const handleSearchReset = (clearFilters: () => void) => {
    clearFilters();
    setSearchText("");
    setSearchedColumn("");
  };

  const getColumnSearchProps = (column: TableColumn<T>): ColumnType<T> => ({
    filterDropdown: ({
      setSelectedKeys,
      selectedKeys,
      confirm,
      clearFilters,
    }) => (
      <div style={{ padding: 8, minWidth: "300px" }}>
        <Input
          ref={searchInput}
          placeholder={`Search ${column.title}`}
          value={selectedKeys[0]}
          onChange={(e) =>
            setSelectedKeys(e.target.value ? [e.target.value] : [])
          }
          onPressEnter={() =>
            handleSearch(
              selectedKeys as string[],
              confirm,
              column.dataIndex as string,
            )
          }
          style={{ marginBottom: 8, display: "block" }}
        />
        <div className="grid grid-cols-3 gap-x-2 items-center">
          <Button
            className="col-span-2"
            type="primary"
            onClick={() =>
              handleSearch(
                selectedKeys as string[],
                confirm,
                column.dataIndex as string,
              )
            }
            size="small"
          >
            Search
          </Button>
          <a
            className="w-full text-center"
            href="javascript:void(0);"
            onClick={() => clearFilters && handleSearchReset(clearFilters)}
          >
            Reset
          </a>
        </div>
      </div>
    ),
    filterIcon: (filtered: boolean) => (
      <SearchOutlined
        style={{ color: filtered ? "#1890ff" : undefined }}
        rev={undefined}
      />
    ),
    onFilter: (value, record) =>
      (column.searchFormatter
        ? column.searchFormatter(record)
        : record[column.dataIndex as string].toString()
      )
        .toLowerCase()
        .includes(value.toString().toLowerCase()),
    onFilterDropdownOpenChange: (visible) => {
      if (visible) {
        setTimeout(() => searchInput.current?.select(), 100);
      }
    },
    render: (text, record) =>
      searchedColumn === column.dataIndex ? (
        column.searchRender !== undefined &&
        column.searchHighlightProps !== undefined ? (
          React.cloneElement(column.searchRender(text, record), {
            [column.searchHighlightProps]: (
              <Highlighter
                highlightStyle={{ backgroundColor: "#ffc069", padding: 0 }}
                searchWords={[searchText]}
                autoEscape
                textToHighlight={
                  column.searchFormatter ? column.searchFormatter(record) : ""
                }
              />
            ),
          })
        ) : (
          <Highlighter
            highlightStyle={{ backgroundColor: "#ffc069", padding: 0 }}
            searchWords={[searchText]}
            autoEscape
            textToHighlight={text ? text.toString() : ""}
          />
        )
      ) : column.searchRender !== undefined ? (
        column.searchRender(text, record)
      ) : (
        text
      ),
  });

  const renderColumn = (column: TableColumn<T>) => {
    let filters: string[] | undefined = undefined;
    if (typeof window !== "undefined" && window.location) {
      const params = new URLSearchParams(window.location.search);
      const currentParam = params.get(column.dataIndex as string);
      if (currentParam) filters = currentParam.split(",");
    }

    if (!column.defaultFilteredValue && filters)
      column.defaultFilteredValue = filters;

    if (column.searchable) {
      return { ...column, ...getColumnSearchProps(column) };
    }
    return {
      ...column,
      filterDropdown: column.filters
        ? ({
            setSelectedKeys,
            selectedKeys,
            confirm,
            filters,
            clearFilters,
          }: FilterDropdownProps) => (
            <div className="container" style={{ minWidth: "170px" }}>
              <div className="container p-2.5 space-y-2">
                {filters &&
                  filters.map((filter, i) => (
                    <div key={i} className="container">
                      <Checkbox
                        onChange={(e) =>
                          setSelectedKeys(
                            e.target.checked
                              ? selectedKeys.concat([filter.value as React.Key])
                              : selectedKeys.filter(
                                  (key) => key !== filter.value,
                                ),
                          )
                        }
                        value={filter.value}
                        checked={selectedKeys.includes(
                          filter.value as React.Key,
                        )}
                      >
                        {filter.text}
                      </Checkbox>
                    </div>
                  ))}
              </div>
              <hr />
              <div className="grid grid-cols-2 p-2.5 justify-items-center items-center">
                <Button size="small" type="primary" onClick={() => confirm()}>
                  OK
                </Button>
                <a
                  href="javascript:void(0);"
                  onClick={() => {
                    if (clearFilters) clearFilters();
                    setSelectedKeys([]);
                  }}
                >
                  Reset
                </a>
              </div>
            </div>
          )
        : undefined,
    };
  };

  interface RowProps extends React.HTMLAttributes<HTMLTableRowElement> {
    "data-row-key": string;
  }

  const RowAntTable = (props: RowProps) => {
    // 只有draggable为true时才初始化拖拽相关逻辑
    const sortable = draggable
      ? useSortable({
          id: props["data-row-key"],
        })
      : null;

    const style: React.CSSProperties = {
      ...props.style,
      // 仅当允许拖动时才添加拖拽相关样式
      ...(draggable
        ? {
            transform: sortable?.transform
              ? CSS.Transform.toString({ ...sortable.transform, scaleY: 1 })
              : undefined,
            transition: sortable?.transition,
            cursor: "move",
            ...(sortable?.isDragging
              ? { position: "relative", zIndex: 9999 }
              : {}),
          }
        : {
            // 禁止拖动时恢复默认光标
            cursor: "default",
          }),
    };

    return (
      <tr
        {...props}
        ref={draggable ? sortable?.setNodeRef : undefined}
        style={style}
        {...(draggable ? sortable?.attributes : {})}
        {...(draggable ? sortable?.listeners : {})}
      />
    );
  };

  /**
   * 行拖拽结束回调
   */
  const onDragEndRow = ({ active, over }: DragEndEvent) => {
    // 仅当允许拖动时才执行排序逻辑
    if (!draggable) return;

    if (active.id !== over?.id) {
      setDataSource((prev) => {
        const activeIndex = prev.findIndex((i) => i.key === active.id);
        const overIndex = prev.findIndex((i) => i.key === over?.id);
        return arrayMove(prev, activeIndex, overIndex);
      });
    }
  };

  /**
   * 列拖拽结束回调
   */
  const onDragEndColumn = (fromIndex: number, toIndex: number) => {
    // 仅当允许拖动时才执行排序逻辑
    if (!draggable) return;

    const columnsCopy = columns.slice();
    const item = columnsCopy.splice(fromIndex, 1)[0];
    columnsCopy.splice(toIndex, 0, item);
    setColumnsData(columnsCopy);

    const order: Order[] = [];
    columnsCopy.forEach((element, index: number) => {
      order.push({ element: element.dataIndex, toIndex: index });
    });
    localStorage.setItem("order", JSON.stringify(order));
  };

  /**
   * 初始化列顺序
   */
  const initializeOrder = () => {
    setColumnsData(initialColumnsData);
    localStorage.setItem("order", "");
  };

  // 抽离Table核心属性，避免重复代码
  const tableProps = {
    components: {
      body: {
        row: RowAntTable,
      },
    },
    rowKey: "key",
    size: "large",
    className: className,
    columns: columns?.map(renderColumn),
    dataSource: dataSource,
    showSorterTooltip: false,
    rowSelection: rowSelection,
    // scroll: { x: true },
    loading: {
      spinning: loading,
      indicator: <AiOutlineLoading className="animate-spin" />,
    },
    pagination: {
      size: "large",
      showSizeChanger: canChangePageSize,
      pageSize: itemsPerPage,
      onShowSizeChange: (current: number, size: number) =>
        setItemsPerPage(size),
      total: dataSource.length,
      position: [paginationPosition],
      showTotal: (total: number) => (
        <div>
          <Row className="margin-right:40px">
            <div>
              <Button
                className=""
                type="primary"
                onClick={() => initializeOrder()}
                size="middle"
                style={{ marginRight: "16px" }}
              >
                Initialize
              </Button>
            </div>
            <div>
              {total > 0 ? (
                <p>
                  {total} item{total > 1 ? "s" : ""}
                </p>
              ) : (
                <p>No item</p>
              )}
            </div>
          </Row>
        </div>
      ),
      style: {
        marginRight: "15px",
        justifyContent: "",
        flex: "",
      },
    },
    expandable: expandable
      ? {
          expandedRowRender,
          rowExpandable,
          expandIcon: ({ expanded, onExpand, record }) =>
            !expanded ? (
              <span
                className="anticon opacity-60 cursor-pointer"
                onClick={(e) => onExpand(record, e)}
              >
                <GoChevronRight />
              </span>
            ) : (
              <span
                className="anticon opacity-60 cursor-pointer"
                onClick={(e) => onExpand(record, e)}
              >
                <GoChevronDown />
              </span>
            ),
        }
      : undefined,
    rowClassName: rowClassName,
  };

  // 渲染带行拖拽的Table
  const renderTableWithRowDrag = () => {
    if (draggable) {
      return (
        <DndContext onDragEnd={onDragEndRow}>
          <SortableContext
            items={dataSource.map((i) => i.key)}
            strategy={verticalListSortingStrategy}
          >
            <AntTable {...tableProps} />
          </SortableContext>
        </DndContext>
      );
    }
    return <AntTable {...tableProps} />;
  };

  return (
    dataSource.length !== 0 && (
      <ConfigProvider renderEmpty={() => <div className="mt-4">Empty</div>}>
        {/* 仅当允许拖动时才包裹列拖拽容器 */}
        {draggable ? (
          <ReactDragListView.DragColumn
            onDragEnd={onDragEndColumn}
            nodeSelector="th"
            enableScroll={false}
          >
            {renderTableWithRowDrag()}
          </ReactDragListView.DragColumn>
        ) : (
          renderTableWithRowDrag()
        )}
      </ConfigProvider>
    )
  );
};

export { DraggableTable };
export type { TableColumn };
