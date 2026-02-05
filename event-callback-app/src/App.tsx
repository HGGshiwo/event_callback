import { WSStatus } from "./components/WSStatus";
import { StateDisplay } from "./components/StateDisplay";
import { ButtonGroup } from "./components/ButtonGroup";
import { AppProvider } from "./context/AppContext";
import { VirtualJoystickGroup } from "./components/VirtualJoystick";
import LogOutputBox from "./components/LogOutputBox/indext";
import { ConfigProvider, type ThemeConfig } from "antd";
import { Table } from "./components/Table";

function App() {
  const theme: ThemeConfig = {
    components: {
      Card: { headerFontSize: "18px" },
    },
  };

  return (
    <AppProvider>
      <ConfigProvider theme={theme}>
        {/* 统一容器：PC端撑满#root，移动端100%宽度 */}
        <div className="w-full h-full flex flex-col gap-4">
          <WSStatus />
          <StateDisplay />
          <ButtonGroup />
          <VirtualJoystickGroup />
          <LogOutputBox />
          {/* <Table /> */}
        </div>
      </ConfigProvider>
    </AppProvider>
  );
}

export default App;
