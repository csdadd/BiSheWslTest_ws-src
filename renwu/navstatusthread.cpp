#include "navstatusthread.h"
#include <QDebug>

NavStatusThread::NavStatusThread(QObject* parent)
    : BaseThread(parent)
{
    m_threadName = "NavStatusThread";
}

NavStatusThread::~NavStatusThread()
{
    stopThread();
}

void NavStatusThread::initialize()
{
    try {
        ROSContextManager::instance().initialize();

        m_rosNode = std::make_shared<rclcpp::Node>("nav_status_node");
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        subscribeROSTopics();
        m_executor->add_node(m_rosNode);

        emit logMessage("NavStatusThread initialized successfully", 0);
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize NavStatusThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}

void NavStatusThread::subscribeROSTopics()
{
    m_navStatusSub = m_rosNode->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status",
        rclcpp::SensorDataQoS(),
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
            processNavigationStatus(msg);
        }
    );

    m_navFeedbackSub = m_rosNode->create_subscription<typename nav2_msgs::action::NavigateToPose::Feedback>(
        "/navigate_to_pose/_action/feedback",
        rclcpp::SensorDataQoS(),
        [this](const typename nav2_msgs::action::NavigateToPose::Feedback::SharedPtr msg) {
            processNavigationFeedback(msg);
        }
    );

    m_navPathSub = m_rosNode->create_subscription<nav_msgs::msg::Path>(
        "/plan",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            processNavigationPath(msg);
        }
    );
}

void NavStatusThread::process()
{
    // qDebug() << "[NavStatusThread] 正在运行 - 获取导航状态、反馈和路径信息";
    if (m_executor && m_rosNode) {
        m_executor->spin_some();
    }
}

void NavStatusThread::cleanup()
{
    m_executor.reset();
    m_rosNode.reset();

    emit connectionStateChanged(false);
    emit logMessage("NavStatusThread cleanup completed", 0);
}

void NavStatusThread::processNavigationStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    if (msg->status_list.empty()) {
        emit navigationStatusReceived(0, "No active navigation task");
        return;
    }

    int status = msg->status_list[0].status;
    QString statusStr;
    QString logMsg;

    switch (status) {
        case 0:
            statusStr = "Pending";
            break;
        case 1:
            statusStr = "Executing";
            logMsg = "Navigation task is executing";
            break;
        case 2:
            statusStr = "Succeeded";
            logMsg = "Navigation task succeeded";
            break;
        case 3:
            statusStr = "Canceled";
            logMsg = "Navigation task canceled";
            break;
        case 4:
            statusStr = "Aborted";
            logMsg = "Navigation task aborted";
            break;
        default:
            statusStr = "Unknown";
            break;
    }

    emit navigationStatusReceived(status, statusStr);

    if (!logMsg.isEmpty()) {
        emit logMessage(logMsg, status >= 3 ? 2 : 0);
    }
}

void NavStatusThread::processNavigationFeedback(const typename nav2_msgs::action::NavigateToPose::Feedback::SharedPtr msg)
{
    double currentX = msg->current_pose.pose.position.x;
    double currentY = msg->current_pose.pose.position.y;
    double goalX = msg->distance_remaining;

    QString feedback = QString("Navigating... Distance remaining: %1m, Current position: (%2, %3)")
                           .arg(goalX, 0, 'f', 2)
                           .arg(currentX, 0, 'f', 2)
                           .arg(currentY, 0, 'f', 2);

    emit navigationFeedbackReceived(feedback);
}

void NavStatusThread::processNavigationPath(const nav_msgs::msg::Path::SharedPtr msg)
{
    QVector<QPointF> path;
    path.reserve(msg->poses.size());

    for (const auto& pose : msg->poses) {
        path.append(QPointF(pose.pose.position.x, pose.pose.position.y));
    }

    emit navigationPathReceived(path);
}
