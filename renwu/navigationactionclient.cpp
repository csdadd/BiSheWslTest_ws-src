#include "navigationactionclient.h"
#include "roscontextmanager.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <QDebug>

NavigationActionClient::NavigationActionClient(QObject* parent)
    : QObject(parent)
    , m_isNavigating(false)
{
    ROSContextManager::instance().initialize();
    m_node = std::make_shared<rclcpp::Node>("navigation_action_client_node");
    m_actionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        m_node, 
        "/navigate_to_pose"
    );
}

NavigationActionClient::~NavigationActionClient()
{
    if (m_isNavigating && m_goalHandle) {
        cancelGoal();
    }
}

bool NavigationActionClient::sendGoal(double x, double y, double yaw)
{
    if (!m_actionClient->wait_for_action_server(std::chrono::seconds(5))) {
        emit goalRejected("Action server not available");
        return false;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = m_node->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    m_currentGoal = goal_msg.pose;

    auto goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    goal_options.goal_response_callback = 
        std::bind(&NavigationActionClient::goalResponseCallback, this, std::placeholders::_1);
    goal_options.feedback_callback = 
        std::bind(&NavigationActionClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    goal_options.result_callback = 
        std::bind(&NavigationActionClient::resultCallback, this, std::placeholders::_1);

    auto goal_future = m_actionClient->async_send_goal(goal_msg, goal_options);
    return true;
}

bool NavigationActionClient::cancelGoal()
{
    if (!m_isNavigating || !m_goalHandle) {
        return false;
    }

    auto cancel_future = m_actionClient->async_cancel_goal(m_goalHandle);
    cancel_future.wait();
    return true;
}

bool NavigationActionClient::isNavigating() const
{
    return m_isNavigating;
}

geometry_msgs::msg::PoseStamped NavigationActionClient::getCurrentGoal() const
{
    return m_currentGoal;
}

void NavigationActionClient::goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle)
{
    if (!goalHandle) {
        m_isNavigating = false;
        emit goalRejected("Goal was rejected by server");
        return;
    }

    m_goalHandle = goalHandle;
    m_isNavigating = true;
    emit goalAccepted();
}

void NavigationActionClient::feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle,
                                              const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    double distanceRemaining = feedback->distance_remaining;
    double navigationTime = feedback->navigation_time.sec + feedback->navigation_time.nanosec / 1e9;
    int recoveries = feedback->number_of_recoveries;

    emit feedbackReceived(distanceRemaining, navigationTime, recoveries);
}

void NavigationActionClient::resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    m_isNavigating = false;
    m_goalHandle.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            emit resultReceived(true, "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            emit resultReceived(false, "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            emit goalCanceled();
            break;
        default:
            emit resultReceived(false, "Unknown result code");
            break;
    }
}
