#include "navigationactionclient.h"
#include "roscontextmanager.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <QDebug>

NavigationActionClient::NavigationActionClient(QObject* parent)
    : QObject(parent)
    , m_isNavigating(false)
    , m_spinTimer(new QTimer(this))
{
    ROSContextManager::instance().initialize();
    m_node = std::make_shared<rclcpp::Node>("navigation_action_client_node");
    m_actionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        m_node,
        "/navigate_to_pose"
    );

    // 启动定时器 spin node，确保 ActionClient 回调被处理
    // 使用多次调用确保处理所有待处理的回调，避免遗漏 feedback
    connect(m_spinTimer, &QTimer::timeout, this, [this]() {
        if (m_node && rclcpp::ok()) {
            // 多次调用以处理所有待处理的回调（spin_some 在回调队列非空时会处理回调）
            rclcpp::spin_some(m_node);
            rclcpp::spin_some(m_node);
            rclcpp::spin_some(m_node);
        }
    });
    m_spinTimer->start(SPIN_PERIOD_MS);

    qDebug() << "[NavigationActionClient] 导航动作客户端初始化完成";
}

NavigationActionClient::~NavigationActionClient()
{
    // 停止定时器
    if (m_spinTimer) {
        m_spinTimer->stop();
    }

    QMutexLocker locker(&m_mutex);

    if (m_isNavigating.load() && m_goalHandle) {
        qWarning() << "[NavigationActionClient] 析构函数中取消导航目标";
        auto cancel_future = m_actionClient->async_cancel_goal(m_goalHandle);
        // 使用超时等待，避免无限阻塞
        cancel_future.wait_for(std::chrono::milliseconds(CANCEL_TIMEOUT_MS));
    }
    qDebug() << "[NavigationActionClient] 导航动作客户端已销毁";
}

bool NavigationActionClient::sendGoal(double x, double y, double yaw)
{
    qInfo() << "[NavigationActionClient] 发送导航目标:" << x << "," << y << "," << yaw;

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
    // 保存 future 以防止 goal 被自动取消
    m_goalFuture = goal_future;
    return true;
}

bool NavigationActionClient::cancelGoal()
{
    QMutexLocker locker(&m_mutex);

    // 只要有 goalHandle 就尝试取消，不依赖 m_isNavigating 状态
    if (!m_goalHandle) {
        qWarning() << "[NavigationActionClient] 尝试取消目标，但没有可用的目标句柄";
        return false;
    }

    qInfo() << "[NavigationActionClient] 正在取消导航目标...";
    auto cancel_future = m_actionClient->async_cancel_goal(m_goalHandle);
    // 使用超时等待，避免无限阻塞 (P0-010修复)
    auto status = cancel_future.wait_for(std::chrono::milliseconds(CANCEL_TIMEOUT_MS));

    if (status == std::future_status::timeout) {
        qWarning() << "[NavigationActionClient] 取消导航目标超时";
        return false;
    }

    return true;
}

bool NavigationActionClient::isNavigating() const
{
    return m_isNavigating.load();
}

geometry_msgs::msg::PoseStamped NavigationActionClient::getCurrentGoal() const
{
    QMutexLocker locker(&m_mutex);
    return m_currentGoal;
}

void NavigationActionClient::goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle)
{
    QMutexLocker locker(&m_mutex);

    if (!goalHandle) {
        m_isNavigating.store(false);
        qCritical() << "[NavigationActionClient] 导航目标被服务器拒绝";
        emit goalRejected("Goal was rejected by server");
        return;
    }

    qInfo() << "[NavigationActionClient] 目标已接受";
    m_goalHandle = goalHandle;
    m_isNavigating.store(true);
    emit goalAccepted();
}

void NavigationActionClient::feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goalHandle,
                                              const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    double distanceRemaining = feedback->distance_remaining;
    double navigationTime = feedback->navigation_time.sec + feedback->navigation_time.nanosec / 1e9;
    int recoveries = feedback->number_of_recoveries;
    double estimatedTimeRemaining = feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec / 1e9;

    // 添加调试日志，确认回调被调用
    static int feedbackCount = 0;
    feedbackCount++;
    // 每次反馈都输出，确保能看到回调被调用
    qDebug() << "[NavigationActionClient] Feedback #" << feedbackCount
             << "剩余距离:" << distanceRemaining
             << "导航时间:" << navigationTime
             << "恢复次数:" << recoveries;

    emit feedbackReceived(distanceRemaining, navigationTime, recoveries, estimatedTimeRemaining);
}

void NavigationActionClient::resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    QMutexLocker locker(&m_mutex);

    m_isNavigating.store(false);
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
