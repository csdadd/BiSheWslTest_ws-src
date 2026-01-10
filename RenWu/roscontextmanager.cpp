#include "roscontextmanager.h"
#include <mutex>

ROSContextManager& ROSContextManager::instance()
{
    static ROSContextManager instance;
    return instance;
}

ROSContextManager::ROSContextManager()
    : m_initialized(false)
{
}

ROSContextManager::~ROSContextManager()
{
    shutdown();
}

void ROSContextManager::initialize()
{
    std::call_once(m_initFlag, [this]() {
        // ROS2 Humble 不提供 is_initialized()，直接尝试初始化
        // 已初始化的上下文可以重复调用 init()
        rclcpp::init(0, nullptr);
        m_context = rclcpp::contexts::get_global_default_context();
        m_initialized.store(true);
    });
}

bool ROSContextManager::isInitialized() const
{
    return m_initialized.load();
}

void ROSContextManager::shutdown()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_initialized.load()) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        m_initialized.store(false);
    }
}

rclcpp::Context::SharedPtr ROSContextManager::getContext() const
{
    return m_context;
}
