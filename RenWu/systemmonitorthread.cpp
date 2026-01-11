#include "systemmonitorthread.h"
#include <QDir>
#include <QCoreApplication>
#include <QDebug>
#include <thread>

SystemMonitorThread::SystemMonitorThread(QObject* parent)
    : BaseThread(parent)
{
    m_threadName = "SystemMonitorThread";
}

SystemMonitorThread::~SystemMonitorThread()
{
    stopThread();
}

void SystemMonitorThread::initialize()
{
    try {
        ROSContextManager::instance().initialize();

        m_rosNode = std::make_shared<rclcpp::Node>("system_monitor_node");
        m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        subscribeROSTopics();
        m_executor->add_node(m_rosNode);
        setExecutor(m_executor);

        m_storageEngine = new LogStorageEngine(this);
        QString dbPath = QCoreApplication::applicationDirPath() + "/logs/system_monitor.db";
        if (!m_storageEngine->initialize(dbPath)) {
            emit threadError(QString("Failed to initialize LogStorageEngine: %1").arg(m_storageEngine->getLastError()));
        }

        emit logMessage("SystemMonitorThread initialized successfully", 0);
        emit connectionStateChanged(true);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize SystemMonitorThread: %1").arg(e.what()));
        emit connectionStateChanged(false);
        throw;
    }
}



void SystemMonitorThread::subscribeROSTopics()
{
    m_rosoutSub = m_rosNode->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        rclcpp::SensorDataQoS(),
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
            processROSLog(msg);
        }
    );

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
    qDebug() << "[SystemMonitorThread] 正在运行 - 监控ROS日志、碰撞检测和行为树";

    QVector<StorageLogEntry> batchEntries;
    MonitorLogEntry entry;
    
    while (m_logQueue.tryDequeue(entry, 0)) {
        emit logMessageReceived(entry.message, entry.level, entry.timestamp);
        
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
    emit logMessage("SystemMonitorThread cleanup completed", 0);
}

void SystemMonitorThread::processROSLog(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    QString message = QString::fromStdString(msg->msg);
    QString source = QString::fromStdString(msg->name);
    int level = MONITOR_LOG_INFO;

    switch (msg->level) {
        case rcl_interfaces::msg::Log::DEBUG:
            level = MONITOR_LOG_DEBUG;
            break;
        case rcl_interfaces::msg::Log::INFO:
            level = MONITOR_LOG_INFO;
            break;
        case rcl_interfaces::msg::Log::WARN:
            level = MONITOR_LOG_WARNING;
            break;
        case rcl_interfaces::msg::Log::ERROR:
            level = MONITOR_LOG_ERROR;
            break;
        case rcl_interfaces::msg::Log::FATAL:
            level = MONITOR_LOG_FATAL;
            break;
        default:
            level = MONITOR_LOG_INFO;
            break;
    }

    QDateTime timestamp;
    timestamp.setSecsSinceEpoch(msg->stamp.sec);
    timestamp.setMSecsSinceEpoch(msg->stamp.nanosec / 1000000);

    m_logQueue.enqueue(MonitorLogEntry(message, level, timestamp, source, "ROS"));

    emit logMessageReceived(message, level, timestamp);

    if (level >= MONITOR_LOG_ERROR) {
        QString logMsg = QString("[%1] %2").arg(source, message);
        emit anomalyDetected(logMsg);
    }
}

void SystemMonitorThread::processCollisionData(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->width * msg->height > 0) {
        QString message = QString("Collision detected! %1 points in bumper cloud").arg(msg->width * msg->height);
        QDateTime timestamp = QDateTime::currentDateTime();

        m_logQueue.enqueue(MonitorLogEntry(message, MONITOR_LOG_ERROR, timestamp, "Bumper", "Collision"));

        emit collisionDetected(message);
        emit logMessageReceived(message, MONITOR_LOG_ERROR, timestamp);
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

            int level = (status == "FAILURE") ? MONITOR_LOG_ERROR : MONITOR_LOG_WARNING;

            m_logQueue.enqueue(MonitorLogEntry(fullMsg, level, timestamp, "BehaviorTree", "Navigation"));

            emit behaviorTreeLogReceived(fullMsg);
            emit logMessageReceived(fullMsg, level, timestamp);

            if (level >= MONITOR_LOG_ERROR) {
                emit anomalyDetected(fullMsg);
            }
        }
    }
}

void SystemMonitorThread::onDiagnosticsReceived(const QString& status, int level, const QString& message)
{
    QDateTime timestamp = QDateTime::currentDateTime();

    int monitorLevel = MONITOR_LOG_INFO;
    // diagnostic_msgs::msg::DiagnosticStatus::OK = 0, WARN = 1, ERROR = 2, STALE = 3
    if (level == 1) {
        monitorLevel = MONITOR_LOG_WARNING;
    } else if (level >= 2) {
        monitorLevel = MONITOR_LOG_ERROR;
    }

    m_logQueue.enqueue(MonitorLogEntry(message, monitorLevel, timestamp, status, "Diagnostics"));

    emit logMessageReceived(message, monitorLevel, timestamp);

    if (level >= 2) {
        QString fullMsg = QString("[%1] %2").arg(status, message);
        emit anomalyDetected(fullMsg);
    }
}

QString SystemMonitorThread::levelToString(int level)
{
    switch (level) {
        case MONITOR_LOG_DEBUG:
            return "DEBUG";
        case MONITOR_LOG_INFO:
            return "INFO";
        case MONITOR_LOG_WARNING:
            return "WARN";
        case MONITOR_LOG_ERROR:
            return "ERROR";
        case MONITOR_LOG_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}
