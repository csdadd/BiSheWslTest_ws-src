#ifndef NAVIGATIONACTIONCLIENT_H
#define NAVIGATIONACTIONCLIENT_H

#include <QObject>
#include <QTimer>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <atomic>
#include <QMutex>

/**
 * @brief 导航动作客户端类
 * @details 管理导航目标的发送、取消和状态跟踪，线程安全
 */
class NavigationActionClient : public QObject
{
    Q_OBJECT

public:
    explicit NavigationActionClient(QObject* parent = nullptr);
    ~NavigationActionClient();

    bool sendGoal(double x, double y, double yaw = 0.0);
    bool cancelGoal();
    bool isNavigating() const;
    geometry_msgs::msg::PoseStamped getCurrentGoal() const;

signals:
    void goalAccepted();
    void goalRejected(const QString& reason);
    void feedbackReceived(double distanceRemaining, double navigationTime, int recoveries, double estimatedTimeRemaining);
    void resultReceived(bool success, const QString& message);
    void goalCanceled();

private:
    void goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle);
    void feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle,
                          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

private:
    void startSpin();
    void stopSpin();

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr m_actionClient;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> m_goalHandle;
    geometry_msgs::msg::PoseStamped m_currentGoal;
    std::atomic<bool> m_isNavigating;
    mutable QMutex m_mutex;
    QTimer* m_spinTimer = nullptr;
    static constexpr int CANCEL_TIMEOUT_MS = 3000;
    static constexpr int SPIN_PERIOD_MS = 50;

    // 保存 goal_future 以防止 goal 被自动取消
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> m_goalFuture;
};

#endif
