#include "navstatusthread.h"
#include <QDebug>

NavStatusThread::NavStatusThread(QObject* parent)
    : BaseThread(parent)
{
    m_threadName = "NavStatusThread";
    qDebug() << "[NavStatusThread] 构造函数";
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

        qDebug() << "[NavStatusThread] 初始化成功";
        emit logMessage("NavStatusThread initialized successfully", LogLevel::INFO);
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize NavStatusThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}

void NavStatusThread::subscribeROSTopics()
{
    if (!m_rosNode) {
        qCritical() << "[NavStatusThread] 错误：ROS节点未初始化";
        emit threadError("ROS node is null, cannot subscribe to topics");
        return;
    }

    m_navStatusSub = m_rosNode->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status",
        rclcpp::SensorDataQoS(),
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
            processNavigationStatus(msg);
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
    m_processCount++;
    if (m_processCount >= 100) {
        // qDebug() << "[NavStatusThread] 正在运行 - 获取导航状态、反馈和路径信息";
        m_processCount = 0;
    }
    if (m_executor && m_rosNode) {
        m_executor->spin_some();
    }
}

void NavStatusThread::cleanup()
{
    m_executor.reset();
    m_rosNode.reset();

    emit connectionStateChanged(false);
    emit logMessage("NavStatusThread cleanup completed", LogLevel::INFO);
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
        emit logMessage(logMsg, status >= 3 ? LogLevel::WARNING : LogLevel::INFO);
    }
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
