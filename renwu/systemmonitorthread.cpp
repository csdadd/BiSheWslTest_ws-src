#include "systemmonitorthread.h"
#include "logutils.h"
#include <QDir>
#include <QCoreApplication>
#include <thread>
#include <QDebug>

SystemMonitorThread::SystemMonitorThread(LogStorageEngine* storageEngine, QObject* parent)
    : BaseThread(parent), m_storageEngine(storageEngine)
{
    m_threadName = "SystemMonitorThread";
    qDebug() << "[SystemMonitorThread] 构造函数";
}

SystemMonitorThread::~SystemMonitorThread()
{
    stopThread();
}

void SystemMonitorThread::initialize()
{
    try {
        emit logMessage("系统初始化中...", LogLevel::INFO);

        ROSContextManager::instance().initialize();

        m_rosNode = std::make_shared<rclcpp::Node>("system_monitor_node");
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        subscribeROSTopics();
        m_executor->add_node(m_rosNode);
        setExecutor(m_executor);

        if (m_storageEngine && !m_storageEngine->isInitialized()) {
            emit threadError("LogStorageEngine not initialized");
        }

        emit logMessage("SystemMonitorThread 初始化成功", LogLevel::INFO);
        emit logMessage("系统监控已启动 - 开始收集日志和诊断信息", LogLevel::INFO);

        qDebug() << "[SystemMonitorThread] 初始化成功";
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize SystemMonitorThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}



void SystemMonitorThread::subscribeROSTopics()
{
    m_rosoutAggSub = m_rosNode->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout_agg",
        rclcpp::SensorDataQoS(),
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
            processROSLog(msg);
        }
    );

    m_collisionSub = m_rosNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mobile_base/sensors/bumper_pointcloud",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            processCollisionData(msg);
        }
    );

    m_behaviorTreeSub = m_rosNode->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
        "/behavior_tree_log",
        rclcpp::SensorDataQoS(),
        [this](const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
            processBehaviorTreeLog(msg);
        }
    );
}

void SystemMonitorThread::process()
{
    m_processCount++;
    if (m_processCount >= 100) {
        // qDebug() << "[SystemMonitorThread] 正在运行 - 监控ROS日志、碰撞检测和行为树";
        m_processCount = 0;
    }

    QVector<StorageLogEntry> batchEntries;
    MonitorLogEntry entry;
    
    while (m_logQueue.tryDequeue(entry, 0)) {
        StorageLogEntry storageEntry(
            entry.message,
            entry.level,
            entry.timestamp,
            entry.source,
            entry.category
        );
        batchEntries.append(storageEntry);
        
        if (batchEntries.size() >= 100) {
            if (m_storageEngine && m_storageEngine->isInitialized()) {
                m_storageEngine->insertLogs(batchEntries);
            }
            batchEntries.clear();
        }
    }
    
    if (!batchEntries.isEmpty() && m_storageEngine && m_storageEngine->isInitialized()) {
        m_storageEngine->insertLogs(batchEntries);
    }
}

void SystemMonitorThread::cleanup()
{
    m_executor.reset();
    m_rosNode.reset();

    emit connectionStateChanged(false);
    emit logMessage("SystemMonitorThread cleanup completed", LogLevel::INFO);
}

void SystemMonitorThread::processROSLog(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    QString message = QString::fromStdString(msg->msg);
    QString source = QString::fromStdString(msg->name);
    LogLevel level = LogLevel::INFO;

    if (source.contains("diagnostic") ||
        message.contains("diagnostic") ||
        message.contains("No status available")) {
        return;
    }

    switch (msg->level) {
        case rcl_interfaces::msg::Log::DEBUG:
            level = LogLevel::DEBUG;
            break;
        case rcl_interfaces::msg::Log::INFO:
            level = LogLevel::INFO;
            break;
        case rcl_interfaces::msg::Log::WARN:
            level = LogLevel::WARNING;
            break;
        case rcl_interfaces::msg::Log::ERROR:
            if (message == "No events recorded." ||
                message == "No status available") {
                level = LogLevel::DEBUG;
            } else {
                level = LogLevel::ERROR;
            }
            break;
        case rcl_interfaces::msg::Log::FATAL:
            level = LogLevel::FATAL;
            break;
        default:
            level = LogLevel::INFO;
            break;
    }

    qint64 totalMSecs = static_cast<qint64>(msg->stamp.sec) * 1000 +
                        msg->stamp.nanosec / 1000000;
    QDateTime timestamp = QDateTime::fromMSecsSinceEpoch(totalMSecs);

    m_logQueue.enqueue(MonitorLogEntry(message, level, timestamp, source, "ROS"));

    emit logMessageReceived(message, static_cast<int>(level), timestamp);

    if (level >= LogLevel::ERROR) {
        QString logMsg = QString("[%1] %2").arg(source, message);
        emit anomalyDetected(logMsg);
    }
}

void SystemMonitorThread::processCollisionData(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->width * msg->height > 0) {
        QString message = QString("Collision detected! %1 points in bumper cloud").arg(msg->width * msg->height);
        QDateTime timestamp = QDateTime::currentDateTime();

        m_logQueue.enqueue(MonitorLogEntry(message, LogLevel::ERROR, timestamp, "Bumper", "Collision"));

        emit collisionDetected(message);
        emit logMessageReceived(message, static_cast<int>(LogLevel::ERROR), timestamp);
    }
}

void SystemMonitorThread::processBehaviorTreeLog(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
{
    for (const auto& event : msg->event_log) {
        QString message = QString::fromStdString(event.node_name);
        QString status = QString::fromStdString(event.current_status);

        if (status == "FAILURE" || status == "RUNNING") {
            QString fullMsg = QString("Behavior Tree: %1 - %2").arg(message, status);
            QDateTime timestamp = QDateTime::currentDateTime();

            LogLevel level = (status == "FAILURE") ? LogLevel::ERROR : LogLevel::WARNING;

            m_logQueue.enqueue(MonitorLogEntry(fullMsg, level, timestamp, "BehaviorTree", "Navigation"));

            emit behaviorTreeLogReceived(fullMsg);
            emit logMessageReceived(fullMsg, static_cast<int>(level), timestamp);

            if (level >= LogLevel::ERROR) {
                emit anomalyDetected(fullMsg);
            }
        }
    }
}

void SystemMonitorThread::onDiagnosticsReceived(const QString& status, int level, const QString& message)
{
    QDateTime timestamp = QDateTime::currentDateTime();

    LogLevel monitorLevel = LogLevel::INFO;
    // diagnostic_msgs::msg::DiagnosticStatus::OK = 0, WARN = 1, ERROR = 2, STALE = 3
    if (level == 1) {
        monitorLevel = LogLevel::WARNING;
    } else if (level >= 2) {
        monitorLevel = LogLevel::ERROR;
    }

    m_logQueue.enqueue(MonitorLogEntry(message, monitorLevel, timestamp, status, "Diagnostics"));

    emit logMessageReceived(message, static_cast<int>(monitorLevel), timestamp);

    if (level >= 2) {
        QString fullMsg = QString("[%1] %2").arg(status, message);
        emit anomalyDetected(fullMsg);
    }
}
