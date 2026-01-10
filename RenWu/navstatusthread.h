#ifndef NAVSTATUSTHREAD_H
#define NAVSTATUSTHREAD_H

#include "basethread.h"
#include "roscontextmanager.h"
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <QPointF>
#include <QVector>

class NavStatusThread : public BaseThread
{
    Q_OBJECT

public:
    explicit NavStatusThread(QObject* parent = nullptr);
    ~NavStatusThread();

signals:
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
    void processNavigationStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
    void processNavigationFeedback(const typename nav2_msgs::action::NavigateToPose::Feedback::SharedPtr msg);
    void processNavigationPath(const nav_msgs::msg::Path::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr m_executor;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr m_navStatusSub;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Feedback>::SharedPtr m_navFeedbackSub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_navPathSub;
};

#endif // NAVSTATUSTHREAD_H
