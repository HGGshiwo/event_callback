#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <typeinfo>
#include <chrono>
#include <mutex>
#include <functional>
#include <type_traits>

// 获取类名（替代Python的get_classname）
template <typename T>
std::string get_classname(const T &obj, bool include_namespace = false)
{
    std::string full_name = typeid(obj).name();
    // 不同编译器的name()格式不同，这里简化实现
    if (!include_namespace)
    {
        size_t last_colon = full_name.find_last_of(':');
        if (last_colon != std::string::npos)
        {
            full_name = full_name.substr(last_colon + 1);
        }
    }
    return full_name;
}

/**
 * 通用限流函数：适配任意参数、任意返回值的可调用对象
 * @tparam Func 可调用对象类型（自动推导）
 * @tparam Args Func的参数类型包（自动推导）
 * @param frequency 调用频率（Hz）
 * @param func 要限流的可调用对象（普通函数、lambda、成员函数等）
 * @return 包装后的限流函数，签名和原函数完全一致
 */
template <typename Func>
auto throttle(double frequency, Func &&func)
{
    // 校验频率合法性
    if (frequency <= 0.0)
    {
        throw std::invalid_argument("Frequency must be positive (frequency > 0)");
    }

    // 1. 定义类型别名：
    // - 推导Func的返回值类型（任意返回值，包括void）
    using ReturnType = std::invoke_result_t<Func>;
    // - 推导Func的调用签名（自动匹配参数类型包）
    using CallSignature = ReturnType(decltype(std::function{func})::argument_type...);

    // 2. 限流核心变量（捕获到lambda中）
    const std::chrono::duration<double> interval(1.0 / frequency);
    std::chrono::time_point<std::chrono::steady_clock> last_call =
        std::chrono::steady_clock::now() - interval;
    std::mutex mtx;

    // 3. 返回包装后的函数（支持任意参数，和原函数签名一致）
    return [func = std::forward<Func>(func),
            interval,
            last_call,
            mtx](auto &&...args) mutable -> ReturnType
    {
        // 线程安全的限流判断
        std::lock_guard<std::mutex> lock(mtx);
        const auto now = std::chrono::steady_clock::now();

        if (now - last_call >= interval)
        {
            last_call = now;
            // 调用原函数，完美转发所有参数（适配任意参数类型/数量）
            return std::invoke(func, std::forward<decltype(args)>(args)...);
        }

        // 处理void返回值（限流时无返回值）
        if constexpr (std::is_void_v<ReturnType>)
        {
            return;
        }
        // 非void返回值：返回默认构造的对象（如int→0，string→""）
        else
        {
            return ReturnType{};
        }
    };
}

#endif // CALLBACK_UTILS_H