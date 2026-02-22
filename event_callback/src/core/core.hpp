#pragma once

#include <any>
#include <functional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>
#include <typeindex>
#include <mutex>
#include <iostream>
#include <type_traits>

// ===================== 基础类型定义 =====================
// 线程局部存储：当前注册的类类型索引
static thread_local std::type_index current_class_idx = std::type_index(typeid(void));

// 组件配置基类（用于统一存储不同组件的配置）
struct ComponentConfig
{
    virtual ~ComponentConfig() = default;
    std::string component_name; // 组件名标识（如"http"）
};

// 前置声明：全局组件注册表
class ComponentRegistry;

// 自动注册标记类（空类，仅用于继承标识）
class AutoRegister
{
public:
    virtual ~AutoRegister() = default;
};

// ===================== CRTP父类：自动注册所有子类 =====================
/**
 * @brief CRTP基类，实现组件的自动注册
 * @tparam Derived 子类类型（CRTP）
 * @tparam ConfigType 子类对应的配置类型（需继承ComponentConfig）
 */
template <typename Derived, typename ConfigType>
class BaseComponent : public AutoRegister
{
public:
    using Ptr = std::shared_ptr<BaseComponent>;
    // 组件构造函数类型：接收配置智能指针，返回组件智能指针
    using ConstructorFunc = std::function<Ptr(const std::shared_ptr<ConfigType> &)>;

    virtual ~BaseComponent() = default;
    // 获取组件名称（纯虚函数，子类必须实现）
    virtual std::string get_component_name() const = 0;
    // 获取静态组件名称（静态函数，子类必须实现）
    static std::string get_static_component_name();

protected:
    // 构造函数：仅子类可调用
    BaseComponent()
    {
        std::cout << "[BaseComponent] 组件构造函数调用（仅实例化时执行）" << std::endl;
    }

private:
    static bool _auto_register_impl();
     __attribute__((used)) static inline bool _auto_register = _auto_register_impl();

    // 友元：允许ComponentRegistry访问私有成员
    friend class ComponentRegistry;
};

// ===================== 全局组件构造函数注册表 =====================
/**
 * @brief 全局组件注册表：存储组件构造函数，实现基于配置的组件实例化
 * 单例模式，线程安全
 */
class ComponentRegistry
{
public:
    // 获取单例实例（懒汉式，线程安全）
    static ComponentRegistry &get_instance()
    {
        static ComponentRegistry instance;
        return instance;
    }

    /**
     * @brief 注册组件构造函数
     * @tparam CompType 组件类型（继承BaseComponent）
     * @tparam ConfigType 配置类型（继承ComponentConfig）
     * @param comp_name 组件名称
     */
    template <typename CompType, typename ConfigType>
    void register_constructor(const std::string &comp_name)
    {
        static_assert(std::is_base_of_v<ComponentConfig, ConfigType>, "ConfigType必须继承ComponentConfig");
        static_assert(std::is_base_of_v<BaseComponent<CompType, ConfigType>, CompType>, "CompType必须继承BaseComponent");

        std::lock_guard<std::mutex> lock(mtx_);
        auto register_cb = [](const std::shared_ptr<ComponentConfig> &config) -> std::shared_ptr<BaseComponent<CompType, ConfigType>>
        {
            auto config_ptr = std::dynamic_pointer_cast<ConfigType>(config);
            if (!config_ptr)
            {
                throw std::invalid_argument("配置类型转换失败");
            }
            return std::make_shared<CompType>(*config_ptr);
        };

        ctor_map_[comp_name] = {comp_name, register_cb};
        std::cout << "[ComponentRegistry] 注册组件构造函数：" << comp_name << "（无实例/配置）" << std::endl;
    }

    /**
     * @brief 根据配置创建组件实例
     * @param config 配置智能指针（any包装）
     * @return 组件智能指针（any包装），失败返回nullptr
     */
    std::shared_ptr<std::any> create_instance(const std::shared_ptr<ComponentConfig> &config)
    {
        if (!config)
        {
            std::cout << "[Error] 配置指针为空，无法实例化组件" << std::endl;
            return nullptr;
        }

        std::lock_guard<std::mutex> lock(mtx_);
        std::string comp_name = config->component_name;
        auto it = ctor_map_.find(comp_name);
        if (it == ctor_map_.end())
        {
            std::cout << "[Error] 配置类型[" << comp_name << "]未注册，无法实例化" << std::endl;
            return nullptr;
        }

        try
        {
            auto [comp_name, ctor] = it->second;
            auto instance = ctor(config);
            std::cout << "[ComponentRegistry] 组件[" << comp_name << "]实例化完成" << std::endl;
            return std::make_shared<std::any>(instance);
        }
        catch (const std::bad_any_cast &e)
        {
            std::cout << "[Error] 配置类型转换失败：" << e.what() << std::endl;
            return nullptr;
        }
        catch (const std::exception &e)
        {
            std::cout << "[Error] 组件实例化失败：" << e.what() << std::endl;
            return nullptr;
        }
    }

private:
    // 禁止拷贝/赋值
    ComponentRegistry() = default;
    ComponentRegistry(const ComponentRegistry &) = delete;
    ComponentRegistry &operator=(const ComponentRegistry &) = delete;

    using CtorData = std::pair<std::string, std::function<std::shared_ptr<AutoRegister>(const std::shared_ptr<ComponentConfig> &)>>;
    std::mutex mtx_;                                     // 线程安全锁
    std::unordered_map<std::string, CtorData> ctor_map_; // 配置类型→构造函数映射
};

// ===================== 全局回调注册表 =====================
/**
 * @brief 全局回调注册表：存储类→组件名→回调函数列表
 * 单例模式，线程安全
 */
class GlobalCallbackRegistry
{
public:
    static GlobalCallbackRegistry &get_instance()
    {
        static GlobalCallbackRegistry instance;
        return instance;
    }

    /**
     * @brief 注册回调函数
     * @tparam ClassType 回调函数接收的参数类型
     * @param class_idx 类类型索引
     * @param comp_name 组件名
     * @param cb 回调函数（接收ClassType类型参数）
     */
    template <typename ClassType>
    void add_callback(std::type_index class_idx, const std::string &comp_name, const std::function<void(ClassType)> &cb)
    {
        if (!cb)
        {
            std::cout << "[Warning] 空回调函数，跳过注册" << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(mtx_);
        // 存储：类索引→组件名→回调列表（any包装）
        std::any cb_any = cb;
        callback_registry_[class_idx][comp_name].push_back(cb_any);
        std::cout << "[GlobalCallbackRegistry] 注册回调到组件名[" << comp_name << "]（无配置）" << std::endl;
    }

    /**
     * @brief 获取指定类的组件名列表
     * @param class_idx 类类型索引
     * @return 组件名列表
     */
    std::vector<std::string> get_component_names(std::type_index class_idx)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<std::string> names;
        auto it = callback_registry_.find(class_idx);
        if (it != callback_registry_.end())
        {
            for (const auto &[name, _] : it->second)
            {
                names.push_back(name);
            }
        }
        return names;
    }

    /**
     * @brief 获取指定类+组件名的回调列表
     * @tparam ClassType 回调函数参数类型
     * @param class_idx 类类型索引
     * @param comp_name 组件名
     * @return 回调函数列表
     */
    template <typename ClassType>
    std::vector<std::function<void(ClassType)>> get_callbacks(std::type_index class_idx, const std::string &comp_name)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<std::function<void(ClassType)>> result;

        auto class_it = callback_registry_.find(class_idx);
        if (class_it == callback_registry_.end())
        {
            return result;
        }

        auto comp_it = class_it->second.find(comp_name);
        if (comp_it == class_it->second.end())
        {
            return result;
        }

        // 转换any为回调函数
        for (const auto &cb_any : comp_it->second)
        {
            try
            {
                auto cb = std::any_cast<std::function<void(ClassType)>>(cb_any);
                if (cb)
                {
                    result.push_back(cb);
                }
            }
            catch (const std::bad_any_cast &e)
            {
                std::cout << "[Warning] 回调类型转换失败：" << e.what() << std::endl;
            }
        }

        return result;
    }

private:
    GlobalCallbackRegistry() = default;
    GlobalCallbackRegistry(const GlobalCallbackRegistry &) = delete;
    GlobalCallbackRegistry &operator=(const GlobalCallbackRegistry &) = delete;

    std::mutex mtx_; // 线程安全锁
    // 存储结构：类索引→组件名→回调列表（any包装）
    std::unordered_map<std::type_index,
                       std::unordered_map<std::string, std::vector<std::any>>>
        callback_registry_;
};

// ===================== 回调管理器 =====================
/**
 * @brief 回调管理器：管理组件配置、实例化、回调绑定
 * @tparam Derived 子类类型（CRTP）
 * @tparam ConfigType 配置类型（继承ComponentConfig）
 */
template <typename Derived>
class CallbackManager
{
public:
    // 静态断言：配置类型必须继承ComponentConfig
    // static_assert(std::is_base_of_v<ComponentConfig, ConfigType>, "ConfigType必须继承ComponentConfig");

    /**
     * @brief 构造函数：初始化配置并实例化组件
     * @param init_configs 初始化配置列表
     */
    explicit CallbackManager(const std::vector<std::shared_ptr<ComponentConfig>> &init_configs)
    {
        current_class_idx = typeid(Derived);
        std::cout << "\n===== CallbackManager构造函数开始 =====" << std::endl;

        // 1. 缓存配置到本地（仅自身使用）
        for (const auto &cfg : init_configs)
        {
            if (cfg)
            {
                component_configs_[cfg->component_name] = cfg;
                std::cout << "[CallbackManager] 缓存配置：组件[" << cfg->component_name << "]（仅自身使用）" << std::endl;
            }
        }

        // 2. 初始化组件（从本地缓存读取配置）
        _init_component();

        std::cout << "===== CallbackManager构造函数结束 =====" << std::endl;
    }

    virtual ~CallbackManager() = default;

    /**
     * @brief 构造后初始化：绑定回调+后置初始化
     */
    void post_construct()
    {
        _bind_and_register_callbacks();
        _post_init();
    }

    /**
     * @brief 创建CallbackManager实例（工厂方法）
     * @param init_configs 初始化配置列表
     * @return 实例智能指针
     */
    static std::shared_ptr<Derived> create(const std::vector<std::shared_ptr<ComponentConfig>> &init_configs)
    {
        auto obj = std::make_shared<Derived>(init_configs);
        obj->post_construct();
        return obj;
    }

protected:
    // 后置初始化（子类可重写）
    virtual void _post_init() {}

    /**
     * @brief 获取组件实例
     * @param comp_name 组件名
     * @return 组件智能指针，不存在返回nullptr
     */
    std::shared_ptr<AutoRegister> get_component(const std::string &comp_name)
    {
        auto it = component_instances_.find(comp_name);
        if (it != component_instances_.end())
        {
            try
            {
                return std::any_cast<std::shared_ptr<AutoRegister>>(*(it->second));
            }
            catch (const std::bad_any_cast &e)
            {
                std::cout << "[Error] 组件类型转换失败：" << e.what() << std::endl;
            }
        }
        return nullptr;
    }

private:
    // 配置缓存：组件名→配置指针
    std::unordered_map<std::string, std::shared_ptr<ComponentConfig>> component_configs_;
    // 组件实例缓存：组件名→实例any指针
    std::unordered_map<std::string, std::shared_ptr<std::any>> component_instances_;

    /**
     * @brief 初始化组件：从本地缓存读取配置，实例化组件
     */
    void _init_component()
    {
        std::cout << "\n===== _init_component开始：实例化组件（仅用自身配置） =====" << std::endl;

        // std::type_index class_idx = typeid(Derived);
        // std::cout << "_init_component: " << class_idx.name() << std::endl;
        // auto comp_names = GlobalCallbackRegistry::get_instance().get_component_names(class_idx);

        for (const auto &[comp_name, config] : component_configs_)
        {
            std::cout << "\n处理组件名：" << comp_name << std::endl;

            // 2. 实例化组件
            auto comp_instance = ComponentRegistry::get_instance().create_instance(config);
            if (comp_instance)
            {
                component_instances_[comp_name] = comp_instance;
            }
        }

        std::cout << "===== _init_component结束：实例化完成 =====" << std::endl;
    }

    /**
     * @brief 绑定回调到组件实例
     */
    void _bind_and_register_callbacks()
    {
        std::cout << "\n===== 为实例化后的组件注册回调 =====" << std::endl;
        std::type_index class_idx = typeid(Derived);
        for (const auto &[comp_name, instance_any_ptr] : component_instances_)
        {
            if (!instance_any_ptr)
            {
                continue;
            }

            // 获取回调列表
            auto callbacks = GlobalCallbackRegistry::get_instance().get_callbacks<std::shared_ptr<AutoRegister>>(class_idx, comp_name);
            if (callbacks.empty())
            {
                std::cout << "[Info] 组件[" << comp_name << "]无回调函数" << std::endl;
                continue;
            }

            // 转换实例类型并执行回调
            try
            {
                auto instance = std::any_cast<std::shared_ptr<AutoRegister>>(*instance_any_ptr);
                for (const auto &callback : callbacks)
                {
                    callback(instance);
                }
                std::cout << "[Info] 组件[" << comp_name << "]绑定" << callbacks.size() << "个回调" << std::endl;
            }
            catch (const std::bad_any_cast &e)
            {
                std::cout << "[Error] 组件实例转换失败：" << e.what() << std::endl;
            }
        }
    }
};

// ===================== BaseComponent 静态成员实现 =====================
template <typename Derived, typename ConfigType>
bool BaseComponent<Derived, ConfigType>::_auto_register_impl()
{
    static_assert(std::is_same_v<decltype(&Derived::get_static_component_name), std::string (*)()>,
                  "子类必须实现静态方法：static std::string get_static_component_name()");
    const std::string comp_name = Derived::get_static_component_name();
    ComponentRegistry::get_instance().register_constructor<Derived, ConfigType>(comp_name);
    std::cout << "[BaseComponent] 自动注册子类[" << typeid(Derived).name() << "]到组件名[" << comp_name << "]（无实例）" << std::endl;
    return true;
}