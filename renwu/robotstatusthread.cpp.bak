#include "robotstatusthread.h"
#include <cmath>
#include <QDebug>

RobotStatusThread::RobotStatusThread(QObject* parent)
    : BaseThread(parent)
    , m_batteryVoltage(0.0f)
{
    m_threadName = "RobotStatusThread";
    qDebug() << "[RobotStatusThread] 构造函数";
}

RobotStatusThread::~RobotStatusThread()
{
    stopThread();
}

void RobotStatusThread::initialize()
{
    try {
        ROSContextManager::instance().initialize();

        m_rosNode = std::make_shared<rclcpp::Node>("robot_status_node");
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        subscribeROSTopics();
        m_executor->add_node(m_rosNode);

        qDebug() << "[RobotStatusThread] 初始化成功";
        emit logMessage("RobotStatusThread initialized successfully", 0);
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize RobotStatusThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}

void RobotStatusThread::subscribeROSTopics()
{
    if (!m_rosNode) {
        qCritical() << "[RobotStatusThread] 错误：ROS节点未初始化";
        emit threadError("ROS node is null, cannot subscribe to topics");
        return;
    }

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
}

void RobotStatusThread::process()
{
    static int count = 0;
    count++;
    if (count >= 100) {
        qDebug() << "[RobotStatusThread] 正在运行 - 获取电池、位置、里程计和诊断信息";
        count = 0;
    }
    if (m_executor && m_rosNode) {
        m_executor->spin_some();
    }

    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    emit systemTimeReceived(currentTime);
}

void RobotStatusThread::cleanup()
{
    m_executor.reset();
    m_rosNode.reset();

    emit connectionStateChanged(false);
    emit logMessage("RobotStatusThread cleanup completed", 0);
}

void RobotStatusThread::processBatteryData(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_batteryVoltage = msg->data;

    float percentage = ((m_batteryVoltage - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY)) * 100.0f;
    percentage = qBound(0.0f, percentage, 100.0f);

    emit batteryStatusReceived(m_batteryVoltage, percentage);

    QString logMsg = QString("Battery: %1V (%2%)").arg(m_batteryVoltage, 0, 'f', 2).arg(percentage, 0, 'f', 1);
    emit logMessage(logMsg, 0);
}

void RobotStatusThread::processPositionData(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
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

void RobotStatusThread::processOdometryData(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void RobotStatusThread::processDiagnosticsData(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    if (msg->status.empty()) {
        emit diagnosticsReceived("Unknown", 0, "No status available");
        return;
    }

    for (const auto& status : msg->status) {
        QString name = QString::fromStdString(status.name);
        int level = status.level;
        QString message = QString::fromStdString(status.message);

        emit diagnosticsReceived(name, level, message);
    }
}

void RobotStatusThread::processTimeData(const sensor_msgs::msg::TimeReference::SharedPtr msg)
{
    QDateTime refTime;
    refTime.setSecsSinceEpoch(msg->header.stamp.sec);
    refTime.setMSecsSinceEpoch(msg->header.stamp.nanosec / 1000000);

    emit systemTimeReceived(refTime.toString("yyyy-MM-dd hh:mm:ss"));
}

double RobotStatusThread::getYawFromQuaternion(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}
