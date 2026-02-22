#pragma once

#include <core.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

template <typename T>
using CallbackFunc = std::tuple<std::string, int, float, std::function<void(T)>>;

// ROS配置（仅CallbackManager使用）
struct ROSConfig : public ComponentConfig
{
    ROSConfig() { component_name = "ros"; }
};

// Dummy ROS组件
class ROSComponent : public BaseComponent<ROSComponent, ROSConfig>
{
public:
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> callbacks;

private:
    ROSConfig config_;

public:
    static std::string get_static_component_name() { return "ros"; }

    explicit ROSComponent()
    {
        config_ = ROSConfig();
        std::cout << "[ROSComponent] 实例初始化" << std::endl;
    }

    explicit ROSComponent(const ROSConfig &config)
        : config_(config)
    {
        std::cout << "[ROSComponent] 实例初始化" << std::endl;
    }

    std::string get_component_name() const override { return get_static_component_name(); }
};

class ROSHelper
{
public:
    // 修复1：修改返回类型为可调用的Lambda类型（std::function）
    template <typename MsgType>
    std::function<void(const std::function<void(MsgType)> &)>
    topic(const std::string &topic, const int queue_size, const float frequency)
    {
        // 修复2：Lambda补充返回类型-> void，语法格式正确
        return [=](const std::function<void(MsgType)> &cb) -> void
        {
            std::function<void(std::shared_ptr<AutoRegister>)> _register_cb = [=](std::shared_ptr<AutoRegister> comp_instance) -> void
            {
                auto ros_comp = std::dynamic_pointer_cast<ROSComponent>(comp_instance);
                if (!ros_comp)
                {
                    std::cout << "[ERROR] ROSComponent 实例不存在!" << std::endl;
                    return;
                };

                boost::function<void(const boost::shared_ptr<MsgType const> &)> ros_cb =
                    [cb](const boost::shared_ptr<MsgType const> &msg)
                { cb(*msg); };
                auto sub = ros_comp->nh.subscribe<MsgType>(topic, queue_size, ros_cb);

                if (!sub)
                {
                    std::cout << "[ERROR]" << topic << "注册失败!" << std::endl;
                    return;
                }
                ros_comp->callbacks.push_back(sub);
                std::cout << "[INFO]" << topic << "注册成功!" << std::endl;
            };

            std::cout << "topic: " << current_class_idx.name() << std::endl;
            GlobalCallbackRegistry::get_instance().add_callback(current_class_idx, "ros", _register_cb);
        };
    }
};

static ROSHelper _ros;
