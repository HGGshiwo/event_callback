import { Card } from "antd";
import { DraggableTable } from "./DraggableTable";
import type { Order, TableColumn } from "./TableColumn";

import { useEffect, useState } from "react";

const columns: TableColumn<any>[] = [
  {
    title: "Key",
    dataIndex: "key",
    width: "5%",
  },
  {
    title: "First Name",
    dataIndex: "firstName",
    width: "20%",
    sorter: (a, b) => a.firstName.localeCompare(b.firstName),
    ellipsis: true,
    searchRender: (text) => <p>{text}</p>,
    searchHighlightProps: "children",
    searchable: true,
    searchFormatter: (record) => record.firstName,
  },
  {
    title: "Last Name",
    dataIndex: "lastName",
    width: "25%",
    sorter: (a, b) => a.lastName.localeCompare(b.lastName),
    ellipsis: true,
    searchRender: (text, record) => <p>{text}</p>,
    searchHighlightProps: "children",
    searchable: true,
    searchFormatter: (record) => record.firstName,
  },
  {
    title: "Email",
    dataIndex: "email",
    width: "25%",
    sorter: (a, b) => a.email.localeCompare(b.email),
    ellipsis: true,
    searchRender: (text, record) => <p>{text}</p>,
    searchHighlightProps: "children",
    searchable: true,
    searchFormatter: (record) => record.email,
  },
  {
    title: "Gender",
    dataIndex: "gender",
    width: "10%",
    sorter: (a, b) => a.gender.localeCompare(b.gender),
    ellipsis: true,
  },
  {
    title: "IP Address",
    dataIndex: "ipAddress",
    width: "15%",
    sorter: (a, b) => a.ipAddress.localeCompare(b.ipAddress),
    ellipsis: true,
  },
];

const data = [
  {
    key: 1,
    firstName: "Clarke",
    lastName: "Smullen",
    email: "csmullen0@shutterfly.com",
    gender: "Male",
    ipAddress: "202.188.190.72",
  },
  {
    key: 2,
    firstName: "Jamill",
    lastName: "Riggs",
    email: "jriggs1@exblog.jp",
    gender: "Male",
    ipAddress: "120.81.10.114",
  },
  {
    key: 3,
    firstName: "Philippa",
    lastName: "Ruske",
    email: "pruske2@soup.io",
    gender: "Female",
    ipAddress: "164.146.53.213",
  },
  {
    key: 4,
    firstName: "Marris",
    lastName: "Edmott",
    email: "medmott3@unc.edu",
    gender: "Female",
    ipAddress: "40.252.142.22",
  },
  {
    key: 5,
    firstName: "Eba",
    lastName: "Pavia",
    email: "epavia4@usgs.gov",
    gender: "Agender",
    ipAddress: "59.118.187.136",
  },
  {
    key: 6,
    firstName: "Solomon",
    lastName: "Doul",
    email: "sdoul5@foxnews.com",
    gender: "Male",
    ipAddress: "103.113.156.220",
  },
  {
    key: 7,
    firstName: "Bobine",
    lastName: "Thys",
    email: "bthys6@tripadvisor.com",
    gender: "Female",
    ipAddress: "178.96.158.239",
  },
  {
    key: 8,
    firstName: "Willie",
    lastName: "Pilbeam",
    email: "wpilbeam7@ebay.com",
    gender: "Male",
    ipAddress: "241.3.112.218",
  },
];
export function Table() {
  const [dataSource, setDataSource] = useState<any[]>([]);
  const [columnsData, setColumnsData] = useState<TableColumn<any>[]>([]);
  const [initialColumnsData, setInitialColumsData] = useState<
    TableColumn<any>[]
  >([]);
  const [isLoading, setIsLoading] = useState(true);

  /**
   * This useEffect aims to patch data and columns to usestates that will later on be passed
   * to the table component.
   */
  useEffect(() => {
    setDataSource(data);
    setColumnsData(columns);
    setInitialColumsData(columns);

    // The order item containes the order previously persisted in the localstorage,
    // If it is not null, then we will reorder the columnsData state
    if (localStorage.getItem("order")) {
      const order: Order[] = JSON.parse(localStorage.getItem("order") || "[]");
      const reorderedList: any = order.map((item) =>
        columns.find((x) => x.dataIndex == item.element),
      );
      setColumnsData(reorderedList);
    }
    setIsLoading(false);
  }, []);

  return dataSource.length != 0 && (
    <Card
      title="表格"
      className="mb-6 w-full"
      variant="borderless"
      style={{ boxShadow: "0 2px 8px rgba(0,0,0,0.08)" }}
    >
      <DraggableTable
        draggable={true}
        columns={columnsData}
        setColumnsData={setColumnsData}
        dataSource={dataSource}
        setDataSource={setDataSource}
        loading={isLoading}
        initialColumnsData={initialColumnsData}
      />
    </Card>
  );
}
