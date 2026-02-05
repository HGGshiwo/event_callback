import type { ColumnType } from "antd/lib/table";
import type { JSX } from "react";

export declare type DataIndex = string | number | (string | number)[];

// We create our own column type, so that we can add more features to each column
export interface TableColumn<T> extends ColumnType<T> {
  title?: string;
  dataIndex: DataIndex;
  searchable?: boolean;
  searchFormatter?: (record: T) => string;
  searchRender?: (searchText: string, record: T) => JSX.Element;
  searchHighlightProps?: string; // On which props generated from searchRender should we highlight ?
}

export interface Order {
  element: DataIndex;
  toIndex: number;
}
