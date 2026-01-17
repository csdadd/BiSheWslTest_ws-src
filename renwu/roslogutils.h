#ifndef ROSLOGUTILS_H
#define ROSLOGUTILS_H

#include "roscontextmanager.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

// 简化的 ROS 日志宏 - 支持 const char* 和 std::string
#define ROS_LOG_INFO(msg) \
    ROS_LOG_INFO_IMPL(msg)

#define ROS_LOG_WARN(msg) \
    ROS_LOG_WARN_IMPL(msg)

#define ROS_LOG_ERROR(msg) \
    ROS_LOG_ERROR_IMPL(msg)

#define ROS_LOG_DEBUG(msg) \
    ROS_LOG_DEBUG_IMPL(msg)

// 实现宏 - 使用 constexpr if 来区分类型
namespace roslogutils {
    inline void log_info(rclcpp::Logger logger, const char* msg) {
        RCLCPP_INFO(logger, "%s", msg);
    }
    inline void log_info(rclcpp::Logger logger, const std::string& msg) {
        RCLCPP_INFO(logger, "%s", msg.c_str());
    }
    inline void log_warn(rclcpp::Logger logger, const char* msg) {
        RCLCPP_WARN(logger, "%s", msg);
    }
    inline void log_warn(rclcpp::Logger logger, const std::string& msg) {
        RCLCPP_WARN(logger, "%s", msg.c_str());
    }
    inline void log_error(rclcpp::Logger logger, const char* msg) {
        RCLCPP_ERROR(logger, "%s", msg);
    }
    inline void log_error(rclcpp::Logger logger, const std::string& msg) {
        RCLCPP_ERROR(logger, "%s", msg.c_str());
    }
    inline void log_debug(rclcpp::Logger logger, const char* msg) {
        RCLCPP_DEBUG(logger, "%s", msg);
    }
    inline void log_debug(rclcpp::Logger logger, const std::string& msg) {
        RCLCPP_DEBUG(logger, "%s", msg.c_str());
    }
}

#define ROS_LOG_INFO_IMPL(msg) \
    roslogutils::log_info(ROSContextManager::instance().getLogger()->get_logger(), msg)

#define ROS_LOG_WARN_IMPL(msg) \
    roslogutils::log_warn(ROSContextManager::instance().getLogger()->get_logger(), msg)

#define ROS_LOG_ERROR_IMPL(msg) \
    roslogutils::log_error(ROSContextManager::instance().getLogger()->get_logger(), msg)

#define ROS_LOG_DEBUG_IMPL(msg) \
    roslogutils::log_debug(ROSContextManager::instance().getLogger()->get_logger(), msg)

#endif // ROSLOGUTILS_H
