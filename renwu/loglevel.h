#ifndef LOGLEVEL_H
#define LOGLEVEL_H

/**
 * @brief 统一的日志级别枚举定义
 *
 * 所有模块共享此日志级别定义，避免重复定义导致的类型不一致问题
 * 使用 enum class 提升类型安全性
 */
enum class LogLevel : int {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    FATAL = 4,
    HIGHFREQ = 5,  // 高频日志级别
    ODOMETRY = 6   // 里程计日志级别
};

// 为了保持向后兼容性，提供内联常量
inline constexpr LogLevel LOG_DEBUG = LogLevel::DEBUG;
inline constexpr LogLevel LOG_INFO = LogLevel::INFO;
inline constexpr LogLevel LOG_WARNING = LogLevel::WARNING;
inline constexpr LogLevel LOG_ERROR = LogLevel::ERROR;
inline constexpr LogLevel LOG_FATAL = LogLevel::FATAL;
inline constexpr LogLevel LOG_HIGHFREQ = LogLevel::HIGHFREQ;
inline constexpr LogLevel LOG_ODOMETRY = LogLevel::ODOMETRY;

#endif // LOGLEVEL_H
