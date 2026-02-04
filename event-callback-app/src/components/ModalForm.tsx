import {
  Modal,
  Form,
  Input,
  InputNumber,
  Button,
  Slider,
  Switch,
  Select,
} from "antd";
import { createRoot } from "react-dom/client";
import { httpRequest } from "../utils";
import { handleButtonClick, type ButtonItemConfig } from "./ButtonGroup";
import { useCallback, useEffect, useState } from "react";

export interface FormItemConfig {
  name: string; // 表单标签名
  id: string; // 表单字段唯一标识
  type: "input" | "slider" | "number" | "switch" | "select";
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  default?: any; // 字段默认值
  max?: number; // 数值/滑块最大值
  min?: number; // 数值/滑块最小值
  step?: number; // 数值/滑块步长
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  options?: Array<{ name: string; [key: string]: any }>; // 下拉框选项
  required?: boolean; // 是否必填（扩展字段）
}

export interface FormConfig {
  config_type: "form";
  title?: string; // 弹窗标题
  method?: string; // 初始值请求方法
  url?: string; // 初始值请求地址
  items: FormItemConfig[]; // 表单项配置
  submit?: ButtonItemConfig; // 确认提交配置
  on_change?: ButtonItemConfig; // 实时提交配置
  initialModalVisible?: boolean; // 弹窗初始显示状态
}

// 核心：表单弹窗组件（所有Hooks都放在这里面）
// eslint-disable-next-line react-refresh/only-export-components
const FormModal = ({
  formConfig,
  onDestroy, // 组件销毁时的回调
}: {
  formConfig: FormConfig;
  onDestroy: () => void;
}) => {
  // 1. 内部管理核心状态
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const [form] = Form.useForm<Record<string, any>>();
  const [modalVisible, setModalVisible] = useState<boolean>(
    formConfig.initialModalVisible ?? true, // 调用时默认显示
  );
  // eslint-disable-next-line @typescript-eslint/no-explicit-any, @typescript-eslint/no-unused-vars
  const [modalFormData, setModalFormData] = useState<Record<string, any>>({});
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const [initialValues, setInitialValues] = useState<Record<string, any>>({});

  // 辅助函数：从FormConfig.items中提取default值
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const getDefaultValues = useCallback((): Record<string, any> => {
    return formConfig.items.reduce(
      (acc, item) => {
        acc[item.id] = item.default ?? "";
        return acc;
      },
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      {} as Record<string, any>,
    );
  }, [formConfig.items]);

  // 2. 处理初始值逻辑：接口获取 或 解析item.default
  useEffect(() => {
    const fetchInitialValues = async () => {
      // 清空之前的状态
      setModalFormData({});
      setInitialValues({});

      // 若配置了url和method，请求接口获取初始值
      if (formConfig.url && formConfig.method) {
        try {
          const res = await httpRequest(formConfig.method, formConfig.url);
          const validData = res || {};
          setInitialValues(validData);
          setModalFormData(validData);
          form.setFieldsValue(validData);
        } catch (error) {
          console.error("获取表单初始值失败：", error);
          // 接口失败时降级使用default值
          const defaultVals = getDefaultValues();
          setInitialValues(defaultVals);
          setModalFormData(defaultVals);
        }
      } else {
        // 无接口时，直接解析item的default值
        const defaultVals = getDefaultValues();
        setInitialValues(defaultVals);
        setModalFormData(defaultVals);
      }
    };

    // 仅在弹窗显示且配置变化时触发
    if (modalVisible) {
      fetchInitialValues();
    }
  }, [
    formConfig.url,
    formConfig.method,
    formConfig.items,
    modalVisible,
    form,
    getDefaultValues,
  ]);

  // 组件卸载时清理
  useEffect(() => {
    return () => {
      form.resetFields();
    };
  }, [form]);

  // 3. 表单值变化处理：更新内部状态 + 实时提交（若配置on_change）

  // 4. 通用提交函数（调用handleButtonClick）
  const submitForm = useCallback(
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    async (params: Record<string, any>, buttonConfig: ButtonItemConfig) => {
      try {
        await handleButtonClick(buttonConfig, params);
      } catch (error) {
        console.error(`提交失败（${buttonConfig.name}）：`, error);
      }
    },
    [],
  );

  const onFormChange = useCallback(
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (changedValues: Record<string, any>, allValues: Record<string, any>) => {
      // 更新内部表单数据状态
      setModalFormData(allValues);

      // 若配置了on_change，立即提交
      if (formConfig.on_change) {
        submitForm(allValues, formConfig.on_change);
      }
    },
    [formConfig.on_change, submitForm],
  );

  // 5. 确认按钮点击逻辑：表单校验 + 提交（若配置submit）
  const onConfirm = useCallback(async () => {
    try {
      // 先做表单校验
      const validValues = await form.validateFields();

      // 若配置了submit，提交表单参数
      if (formConfig.submit) {
        await submitForm(validValues, formConfig.submit);
      }

      // 提交成功后关闭弹窗并销毁组件
      setModalVisible(false);
      onDestroy();
    } catch (error) {
      console.error("表单校验/提交失败：", error);
    }
  }, [form, formConfig.submit, submitForm, onDestroy]);

  // 6. 取消按钮点击逻辑：关闭弹窗并销毁组件
  const onCancel = useCallback(() => {
    setModalVisible(false);
    onDestroy();
  }, [onDestroy]);

  // 7. 渲染表单控件（根据type匹配对应组件）
  const getInput = useCallback((item: FormItemConfig) => {
    switch (item.type) {
      case "input":
        return <Input placeholder={`请输入${item.name}`} />;
      case "number":
        return (
          <InputNumber
            max={item.max}
            min={item.min}
            step={item.step}
            placeholder={`请输入${item.name}`}
            style={{ width: "100%" }}
          />
        );
      case "slider":
        return <Slider max={item.max} min={item.min} step={item.step} />;
      case "switch":
        return <Switch />;
      case "select":
        return (
          <Select
            options={
              Object.entries(item.options || {}).map(([key, option]) => ({
                value: key,
                label: option,
              })) || []
            }
            placeholder={`请选择${item.name}`}
          />
        );
      default:
        return null;
    }
  }, []);

  // 8. 渲染单个表单项
  const renderFormItem = useCallback(
    (key: string, item: FormItemConfig) => {
      return (
        <Form.Item
          key={key}
          label={item.name}
          name={key}
          rules={
            item.required !== false
              ? [{ required: true, message: `请输入/选择${item.name}` }]
              : []
          }
          initialValue={initialValues[key]}
        >
          {getInput(item)}
        </Form.Item>
      );
    },
    [getInput, initialValues],
  );

  // 弹窗标题默认值
  const modalTitle = formConfig.title;

  // 渲染最终组件
  return (
    <Modal
      title={modalTitle}
      open={modalVisible}
      onCancel={onCancel}
      footer={[
        <Button key="cancel" onClick={onCancel}>
          取消
        </Button>,
        <Button
          key="confirm"
          type="primary"
          onClick={onConfirm}
          disabled={!formConfig.submit} // 无submit配置时禁用确认按钮
        >
          {formConfig.submit?.name || "确定"}
        </Button>,
      ]}
      width={window.innerWidth < 768 ? "90%" : 500} // 移动端自适应
      destroyOnHidden // 关闭弹窗时销毁表单，避免缓存
      maskClosable={false} // 点击遮罩层不关闭，防止误操作
    >
      <Form
        form={form}
        onValuesChange={onFormChange}
        initialValues={initialValues}
        preserve={false}
        layout="vertical"
      >
        {Object.entries(formConfig.items).map(([key, item]) =>
          renderFormItem(key, item),
        )}
      </Form>
    </Modal>
  );
};

/**
 * 触发表单弹窗显示的函数（无Hooks，仅负责动态挂载组件）
 * @param formConfig 完整的表单配置
 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any, @typescript-eslint/no-unused-vars
export const renderForm = (formConfig: FormConfig, params: any) => {
  // 1. 检查并创建唯一的DOM容器
  let container = document.getElementById(`form-modal`);

  // 如果已存在，先销毁旧的组件
  if (container) {
    const root = createRoot(container);
    root.unmount();
    container.remove();
  }

  // 2. 创建新的DOM容器并添加到body
  container = document.createElement("div");
  container.id = `form-modal`;
  document.body.appendChild(container);

  // 3. 定义组件销毁回调
  const destroyComponent = () => {
    const root = createRoot(container!);
    root.unmount();
    container!.remove();
  };

  // 4. 动态挂载FormModal组件
  const root = createRoot(container);
  root.render(
    <FormModal formConfig={formConfig} onDestroy={destroyComponent} />,
  );
};
