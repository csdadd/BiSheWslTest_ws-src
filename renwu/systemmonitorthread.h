#ifndef SYSTEMMONITORTHREAD_H
#define SYSTEMMONITORTHREAD_H

#include "basethread.h"
#include "roscontextmanager.h"
#include "loglevel.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <QDateTime>

// 使用统一的LogLevel定义
using MonitorLogLevel = LogLevel;

struct MonitorLogEntry {
    QString message;
    LogLevel level;
    QDateTime timestamp;
    QString source;

    MonitorLogEntry() : level(LogLevel::INFO) {}
    MonitorLogEntry(const QString& msg, LogLevel lvl, const QDateTime& ts,
                    const QString& src = "")
        : message(msg), level(lvl), timestamp(ts), source(src) {}
};

class SystemMonitorThread : public BaseThread
{
    Q_OBJECT

public:
    explicit SystemMonitorThread(QObject* parent = nullptr);
    ~SystemMonitorThread();

public slots:
    void onDiagnosticsReceived(const QString& status, int level, const QString& message);

signals:
    void logMessageReceived(const QString& message, int level, const QDateTime& timestamp);
    void collisionDetected(const QString& message);
    void anomalyDetected(const QString& message);
    void behaviorTreeLogReceived(const QString& log);
    void connectionStateChanged(bool connected);

protected:
    void initialize() override;
    void process() override;
    void cleanup() override;

private:
    void subscribeROSTopics();
    void processROSLog(const rcl_interfaces::msg::Log::SharedPtr msg);
    void processCollisionData(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processBehaviorTreeLog(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr m_executor;

    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr m_rosoutAggSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_collisionSub;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr m_behaviorTreeSub;

    int m_processCount = 0;
    int m_nav2ActiveCount = 0;  // Nav2 is active 日志采样计数器
};

#endif // SYSTEMMONITORTHREAD_H
