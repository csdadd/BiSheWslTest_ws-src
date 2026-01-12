#ifndef NAVIGATIONACTIONCLIENT_H
#define NAVIGATIONACTIONCLIENT_H

#include <QObject>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

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
    void feedbackReceived(double distanceRemaining, double navigationTime, int recoveries);
    void resultReceived(bool success, const QString& message);
    void goalCanceled();

private:
    void goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle);
    void feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle,
                          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr m_actionClient;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> m_goalHandle;
    geometry_msgs::msg::PoseStamped m_currentGoal;
    bool m_isNavigating;
};

#endif
