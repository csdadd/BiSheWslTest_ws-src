#ifndef INFOTHREAD_H
#define INFOTHREAD_H

#include "basethread.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <QTimer>
#include <QDateTime>
#include <QPointF>
#include <QVector>

class InfoThread : public BaseThread
{
    Q_OBJECT

public:
    explicit InfoThread(QObject* parent = nullptr);
    ~InfoThread();

signals:
    void batteryStatusReceived(float voltage, float percentage);
    void positionReceived(double x, double y, double yaw);
    void odometryReceived(double x, double y, double yaw, double vx, double vy, double omega);
    void systemStatusReceived(const QString& status);
    void systemTimeReceived(const QString& time);
    void navigationStatusReceived(int status, const QString& message);
    void navigationFeedbackReceived(const QString& feedback);
    void navigationPathReceived(const QVector<QPointF>& path);
    void connectionStateChanged(bool connected);

protected:
    void initialize() override;
    void process() override;
    void cleanup() override;

private:
    void subscribeROSTopics();
    void processBatteryData(const std_msgs::msg::Float32::SharedPtr msg);
    void processPositionData(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void processOdometryData(const nav_msgs::msg::Odometry::SharedPtr msg);
    void processDiagnosticsData(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
    void processTimeData(const sensor_msgs::msg::TimeReference::SharedPtr msg);
    void processNavigationStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    void processNavigationFeedback(const typename nav2_msgs::action::NavigateToPose::Feedback::SharedPtr msg);
    void processNavigationPath(const nav_msgs::msg::Path::SharedPtr msg);
    double getYawFromQuaternion(double x, double y, double z, double w);

private:
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr m_executor;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_batterySub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_positionSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometrySub;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnosticsSub;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr m_timeRefSub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr m_navStatusSub;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Feedback>::SharedPtr m_navFeedbackSub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_navPathSub;

    QDateTime m_startTime;
    float m_batteryVoltage;
    static constexpr float BATTERY_FULL = 12.6f;
    static constexpr float BATTERY_EMPTY = 10.0f;
};

#endif // INFOTHREAD_H
