#include "infothread.h"
#include <cmath>
#include <QtConcurrent>

InfoThread::InfoThread(QObject* parent)
    : BaseThread(parent)
    , m_batteryVoltage(0.0f)
{
    m_threadName = "InfoThread";
}

InfoThread::~InfoThread()
{
    stopThread();
}

void InfoThread::initialize()
{
    try {
        rclcpp::init(0, nullptr);

        m_rosNode = std::make_shared<rclcpp::Node>("info_thread_node");
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        subscribeROSTopics();

        m_executor->add_node(m_rosNode);

        m_startTime = QDateTime::currentDateTime();

        emit logMessage("InfoThread initialized successfully", 0);
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize InfoThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}

void InfoThread::subscribeROSTopics()
{
    m_batterySub = m_rosNode->create_subscription<std_msgs::msg::Float32>(
        "/PowerVoltage",
        rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            processBatteryData(msg);
        }
    );

    m_positionSub = m_rosNode->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            processPositionData(msg);
        }
    );

    m_odometrySub = m_rosNode->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            processOdometryData(msg);
        }
    );

    m_diagnosticsSub = m_rosNode->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics",
        rclcpp::SensorDataQoS(),
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
            processDiagnosticsData(msg);
        }
    );

    m_timeRefSub = m_rosNode->create_subscription<sensor_msgs::msg::TimeReference>(
        "/time_reference",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::TimeReference::SharedPtr msg) {
            processTimeData(msg);
        }
    );

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

void InfoThread::process()
{
    if (m_executor && m_rosNode) {
        m_executor->spin_some();
    }

    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    emit systemTimeReceived(currentTime);
}

void InfoThread::cleanup()
{
    m_executor.reset();
    m_rosNode.reset();
    rclcpp::shutdown();

    emit connectionStateChanged(false);
    emit logMessage("InfoThread cleanup completed", 0);
}

void InfoThread::processBatteryData(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_batteryVoltage = msg->data;

    float percentage = ((m_batteryVoltage - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY)) * 100.0f;
    percentage = qBound(0.0f, percentage, 100.0f);

    emit batteryStatusReceived(m_batteryVoltage, percentage);

    QString logMsg = QString("Battery: %1V (%2%)").arg(m_batteryVoltage, 0, 'f', 2).arg(percentage, 0, 'f', 1);
    emit logMessage(logMsg, 0);
}

void InfoThread::processPositionData(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = getYawFromQuaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    emit positionReceived(x, y, yaw);
}

void InfoThread::processOdometryData(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = getYawFromQuaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double omega = msg->twist.twist.angular.z;

    emit odometryReceived(x, y, yaw, vx, vy, omega);
}

void InfoThread::processDiagnosticsData(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    if (msg->status.empty()) {
        emit systemStatusReceived("Unknown");
        return;
    }

    const auto& status = msg->status[0];
    QString levelStr;
    int level = status.level;

    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::OK:
            levelStr = "OK";
            break;
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            levelStr = "Warning";
            break;
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            levelStr = "Error";
            break;
        case diagnostic_msgs::msg::DiagnosticStatus::STALE:
            levelStr = "Stale";
            break;
        default:
            levelStr = "Unknown";
            break;
    }

    QString statusMsg = QString("%1: %2").arg(levelStr, QString::fromStdString(status.name));
    emit systemStatusReceived(statusMsg);

    if (level > 0) {
        emit logMessage(QString("Diagnostics: %1 - %2").arg(statusMsg, QString::fromStdString(status.message)), level);
    }
}

void InfoThread::processTimeData(const sensor_msgs::msg::TimeReference::SharedPtr msg)
{
    QDateTime refTime;
    refTime.setSecsSinceEpoch(msg->header.stamp.sec);
    refTime.setMSecsSinceEpoch(msg->header.stamp.nanosec / 1000000);

    emit systemTimeReceived(refTime.toString("yyyy-MM-dd hh:mm:ss"));
}

void InfoThread::processNavigationStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
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

void InfoThread::processNavigationFeedback(const typename nav2_msgs::action::NavigateToPose::Feedback::SharedPtr msg)
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

void InfoThread::processNavigationPath(const nav_msgs::msg::Path::SharedPtr msg)
{
    QVector<QPointF> path;
    path.reserve(msg->poses.size());

    for (const auto& pose : msg->poses) {
        path.append(QPointF(pose.pose.position.x, pose.pose.position.y));
    }

    emit navigationPathReceived(path);
}

double InfoThread::getYawFromQuaternion(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}
