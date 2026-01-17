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
    qDebug() << "[NavigationActionClient] 导航动作客户端初始化完成";
}

NavigationActionClient::~NavigationActionClient()
{
    if (m_isNavigating && m_goalHandle) {
        qWarning() << "[NavigationActionClient] 析构函数中取消导航目标";
        cancelGoal();
    }
    qDebug() << "[NavigationActionClient] 导航动作客户端已销毁";
}

bool NavigationActionClient::sendGoal(double x, double y, double yaw)
{
    if (!m_actionClient->wait_for_action_server(std::chrono::seconds(5))) {
        qCritical() << "[NavigationActionClient] 导航服务器不可用";
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

    qDebug() << "[NavigationActionClient] 发送导航目标 - X:" << x << ", Y:" << y << ", Yaw:" << yaw;

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
        qWarning() << "[NavigationActionClient] 尝试取消目标，但当前没有活动导航";
        return false;
    }

    qDebug() << "[NavigationActionClient] 正在取消导航目标";
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
        qCritical() << "[NavigationActionClient] 导航目标被服务器拒绝";
        emit goalRejected("Goal was rejected by server");
        return;
    }

    m_goalHandle = goalHandle;
    m_isNavigating = true;
    qDebug() << "[NavigationActionClient] 导航目标已被接受";
    emit goalAccepted();
}

void NavigationActionClient::feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle,
                                              const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    double distanceRemaining = feedback->distance_remaining;
    double navigationTime = feedback->navigation_time.sec + feedback->navigation_time.nanosec / 1e9;
    int recoveries = feedback->number_of_recoveries;

    qDebug() << "[NavigationActionClient] 导航反馈 - 剩余距离:" << distanceRemaining << "m, 已用时间:" << navigationTime << "s, 恢复次数:" << recoveries;

    emit feedbackReceived(distanceRemaining, navigationTime, recoveries);
}

void NavigationActionClient::resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    m_isNavigating = false;
    m_goalHandle.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            qDebug() << "[NavigationActionClient] 导航成功完成";
            emit resultReceived(true, "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            qCritical() << "[NavigationActionClient] 导航被中止";
            emit resultReceived(false, "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            qDebug() << "[NavigationActionClient] 导航已取消";
            emit goalCanceled();
            break;
        default:
            qCritical() << "[NavigationActionClient] 未知导航结果代码";
            emit resultReceived(false, "Unknown result code");
            break;
    }
}
