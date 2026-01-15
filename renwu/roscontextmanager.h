#ifndef ROSCONTEXTMANAGER_H
#define ROSCONTEXTMANAGER_H

// 必须在任何 Qt 头文件之前包含 ROS2 头文件
// 因为 Qt6 的 QFile 包含 <filesystem>，会与 GCC 11 的 std::wstring_convert 冲突
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <mutex>
#include <memory>

class ROSContextManager
{
public:
    static ROSContextManager& instance();

    void initialize();
    bool isInitialized() const;
    void shutdown();

    rclcpp::Context::SharedPtr getContext() const;

    ROSContextManager(const ROSContextManager&) = delete;
    ROSContextManager& operator=(const ROSContextManager&) = delete;

private:
    ROSContextManager();
    ~ROSContextManager();

    std::once_flag m_initFlag;
    rclcpp::Context::SharedPtr m_context;
    mutable std::mutex m_mutex;
    std::atomic<bool> m_initialized;
};

#endif // ROSCONTEXTMANAGER_H
