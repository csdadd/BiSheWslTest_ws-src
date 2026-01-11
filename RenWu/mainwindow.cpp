#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QFileDialog>
#include <algorithm>
#include "maploader.h"
#include "mapconverter.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_robotStatusThread(nullptr)
    , m_navStatusThread(nullptr)
    , m_systemMonitorThread(nullptr)
    , m_logThread(nullptr)
    , m_logTableModel(nullptr)
    , m_logFilterProxyModel(nullptr)
    , m_mapThread(nullptr)
    , m_mapWidget(nullptr)
    , m_mapCache(nullptr)
{
    ui->setupUi(this);

    m_logTableModel = new LogTableModel(this);
    m_logFilterProxyModel = new LogFilterProxyModel(this);
    m_logFilterProxyModel->setSourceModel(m_logTableModel);
    ui->logTableView->setModel(m_logFilterProxyModel);

    ui->logTableView->setColumnWidth(0, 180);
    ui->logTableView->setColumnWidth(1, 80);
    ui->logTableView->setColumnWidth(2, 120);
    ui->logTableView->horizontalHeader()->setStretchLastSection(true);

    m_threadPool = new QThreadPool(this);
    m_threadPool->setMaxThreadCount(2);

    ui->startTimeEdit->setDateTime(QDateTime::currentDateTime().addDays(-1));
    ui->endTimeEdit->setDateTime(QDateTime::currentDateTime());

    m_mapWidget = new MapWidget(this);
    ui->mapLayout->addWidget(m_mapWidget);

    m_mapCache = new MapCache(10, this);

    initializeThreads();
    connectSignals();
    startAllThreads();
}

MainWindow::~MainWindow()
{
    stopAllThreads();

    delete m_robotStatusThread;
    delete m_navStatusThread;
    delete m_systemMonitorThread;
    delete m_logThread;
    delete m_mapThread;
    delete m_mapWidget;
    delete m_mapCache;
    delete ui;
}

void MainWindow::initializeThreads()
{
    m_robotStatusThread = new RobotStatusThread(this);
    m_navStatusThread = new NavStatusThread(this);
    m_systemMonitorThread = new SystemMonitorThread(this);
    m_logThread = new LogThread(this);
    m_mapThread = new MapThread(this);

    m_logTableModel->setStorageEngine(m_logThread->getStorageEngine());
}

void MainWindow::connectSignals()
{
    // RobotStatusThread信号连接
    connect(m_robotStatusThread, &RobotStatusThread::batteryStatusReceived,
            this, &MainWindow::onBatteryStatusReceived);
    connect(m_robotStatusThread, &RobotStatusThread::positionReceived,
            this, &MainWindow::onPositionReceived);
    connect(m_robotStatusThread, &RobotStatusThread::odometryReceived,
            this, &MainWindow::onOdometryReceived);
    connect(m_robotStatusThread, &RobotStatusThread::systemTimeReceived,
            this, &MainWindow::onSystemTimeReceived);
    connect(m_robotStatusThread, &RobotStatusThread::diagnosticsReceived,
            this, &MainWindow::onDiagnosticsReceived);
    connect(m_robotStatusThread, &RobotStatusThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // RobotStatusThread日志转发给LogThread
    connect(m_robotStatusThread, &RobotStatusThread::logMessage,
            m_logThread, &LogThread::writeLog);

    // NavStatusThread信号连接
    connect(m_navStatusThread, &NavStatusThread::navigationStatusReceived,
            this, &MainWindow::onNavigationStatusReceived);
    connect(m_navStatusThread, &NavStatusThread::navigationFeedbackReceived,
            this, &MainWindow::onNavigationFeedbackReceived);
    connect(m_navStatusThread, &NavStatusThread::navigationPathReceived,
            this, &MainWindow::onNavigationPathReceived);
    connect(m_navStatusThread, &NavStatusThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // NavStatusThread日志转发给LogThread
    connect(m_navStatusThread, &NavStatusThread::logMessage,
            m_logThread, &LogThread::writeLog);

    // SystemMonitorThread信号连接
    connect(m_systemMonitorThread, &SystemMonitorThread::logMessageReceived,
            this, &MainWindow::onLogMessageReceived);
    connect(m_systemMonitorThread, &SystemMonitorThread::collisionDetected,
            this, &MainWindow::onCollisionDetected);
    connect(m_systemMonitorThread, &SystemMonitorThread::anomalyDetected,
            this, &MainWindow::onAnomalyDetected);
    connect(m_systemMonitorThread, &SystemMonitorThread::behaviorTreeLogReceived,
            this, &MainWindow::onBehaviorTreeLogReceived);
    connect(m_systemMonitorThread, &SystemMonitorThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // SystemMonitorThread日志转发给LogThread
    connect(m_systemMonitorThread, &SystemMonitorThread::logMessage,
            m_logThread, &LogThread::writeLog);

    // SystemMonitorThread日志接收后转发给LogThread
    connect(m_systemMonitorThread, &SystemMonitorThread::logMessageReceived,
            m_logThread, [this](const QString& message, int level, const QDateTime& timestamp) {
        LogEntry entry(message, level, timestamp, "SystemMonitor", "ROS");
        m_logThread->writeLogEntry(entry);
    });

    // RobotStatusThread的诊断信息转发给SystemMonitorThread
    connect(m_robotStatusThread, &RobotStatusThread::diagnosticsReceived,
            m_systemMonitorThread, &SystemMonitorThread::onDiagnosticsReceived);

    // LogThread信号连接
    connect(m_logThread, &LogThread::logFileChanged,
            this, &MainWindow::onLogFileChanged);

    // MapThread信号连接
    connect(m_mapThread, &MapThread::mapReceived,
            this, &MainWindow::onMapReceived);
    connect(m_mapThread, &MapThread::connectionStateChanged,
            this, &MainWindow::onMapConnectionStateChanged);

    // RobotStatusThread -> MapWidget
    connect(m_robotStatusThread, &RobotStatusThread::positionReceived,
            this, [this](double x, double y, double yaw) {
                if (m_mapWidget) {
                    m_mapWidget->updateRobotPose(x, y, yaw);
                }
            });

    // MapWidget -> MainWindow
    connect(m_mapWidget, &MapWidget::mapClicked,
            this, &MainWindow::onMapClicked);

    // 线程状态信号连接
    connect(m_robotStatusThread, &RobotStatusThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_robotStatusThread, &RobotStatusThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_robotStatusThread, &RobotStatusThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_navStatusThread, &NavStatusThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_navStatusThread, &NavStatusThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_navStatusThread, &NavStatusThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_systemMonitorThread, &SystemMonitorThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_systemMonitorThread, &SystemMonitorThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_systemMonitorThread, &SystemMonitorThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_logThread, &LogThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_logThread, &LogThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_logThread, &LogThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_mapThread, &MapThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_mapThread, &MapThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_mapThread, &MapThread::threadError,
            this, &MainWindow::onThreadError);

    // 连接过滤控件信号
    connect(ui->debugCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->infoCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->warningCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->errorCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->fatalCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->keywordLineEdit, &QLineEdit::textChanged, this, &MainWindow::onFilterChanged);
    connect(ui->sourceLineEdit, &QLineEdit::textChanged, this, &MainWindow::onFilterChanged);
    connect(ui->startTimeEdit, &QDateTimeEdit::dateTimeChanged, this, &MainWindow::onFilterChanged);
    connect(ui->endTimeEdit, &QDateTimeEdit::dateTimeChanged, this, &MainWindow::onFilterChanged);

    // 连接按钮信号
    connect(ui->queryButton, &QPushButton::clicked, this, &MainWindow::onQueryButtonClicked);
    connect(ui->clearFilterButton, &QPushButton::clicked, this, &MainWindow::onClearFilterButtonClicked);
    connect(ui->refreshButton, &QPushButton::clicked, this, &MainWindow::onRefreshButtonClicked);
    connect(ui->loadMapButton, &QPushButton::clicked, this, &MainWindow::onLoadMapFromFile);
}

void MainWindow::startAllThreads()
{
    // 启动顺序：日志 -> 系统监控 -> 机器人状态 -> 导航状态 -> 地图
    m_logThread->start();
    QThread::msleep(100);

    m_systemMonitorThread->start();
    QThread::msleep(100);

    m_robotStatusThread->start();
    QThread::msleep(100);

    m_navStatusThread->start();
    QThread::msleep(100);

    m_mapThread->start();
}

void MainWindow::stopAllThreads()
{
    // 停止顺序与启动相反
    if (m_mapThread && m_mapThread->isRunning()) {
        m_mapThread->stopThread();
        m_mapThread->wait(3000);
    }

    if (m_navStatusThread && m_navStatusThread->isRunning()) {
        m_navStatusThread->stopThread();
        m_navStatusThread->wait(3000);
    }

    if (m_robotStatusThread && m_robotStatusThread->isRunning()) {
        m_robotStatusThread->stopThread();
        m_robotStatusThread->wait(3000);
    }

    if (m_systemMonitorThread && m_systemMonitorThread->isRunning()) {
        m_systemMonitorThread->stopThread();
        m_systemMonitorThread->wait(3000);
    }

    if (m_logThread && m_logThread->isRunning()) {
        m_logThread->stopThread();
        m_logThread->wait(3000);
    }
}

// ==================== RobotStatusThread槽函数 ====================

void MainWindow::onBatteryStatusReceived(float voltage, float percentage)
{
    QString message = QString("电池状态 - 电压: %1 V, 电量: %2%").arg(voltage, 0, 'f', 2).arg(percentage, 0, 'f', 1);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Battery");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onPositionReceived(double x, double y, double yaw)
{
    QString message = QString("位置信息 - X: %1, Y: %2, Yaw: %3").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Position");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega)
{
    QString message = QString("里程计 - 位置(X:%1, Y:%2, Yaw:%3), 速度(vx:%4, vy:%5, omega:%6)")
        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2)
        .arg(vx, 0, 'f', 2).arg(vy, 0, 'f', 2).arg(omega, 0, 'f', 2);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Odometry");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onSystemTimeReceived(const QString& time)
{
    ui->statusbar->showMessage(QString("系统时间: %1").arg(time));
}

void MainWindow::onDiagnosticsReceived(const QString& status, int level, const QString& message)
{
    int logLevel = LOG_INFO;
    if (level == 1) logLevel = LOG_WARNING;
    else if (level == 2) logLevel = LOG_ERROR;
    else if (level == 3) logLevel = LOG_FATAL;
    else if (level == 4) logLevel = LOG_DEBUG;

    QString logMessage = QString("诊断信息 - 状态: %1, 消息: %2").arg(status).arg(message);
    LogEntry entry(logMessage, logLevel, QDateTime::currentDateTime(), "RobotStatus", "Diagnostics");
    m_logTableModel->addLogEntry(entry);

    if (level >= 2) {
        ui->statusbar->showMessage(QString("诊断警告: %1").arg(message), 5000);
    }
}

// ==================== NavStatusThread槽函数 ====================

void MainWindow::onNavigationStatusReceived(int status, const QString& message)
{
    QString statusStr;
    switch (status) {
        case 0: statusStr = "空闲"; break;
        case 1: statusStr = "规划中"; break;
        case 2: statusStr = "执行中"; break;
        case 3: statusStr = "完成"; break;
        case 4: statusStr = "失败"; break;
        default: statusStr = QString::number(status); break;
    }

    QString logMessage = QString("导航状态 - %1: %2").arg(statusStr).arg(message);
    LogEntry entry(logMessage, LOG_INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("导航: %1 - %2").arg(statusStr).arg(message), 3000);
}

void MainWindow::onNavigationFeedbackReceived(const QString& feedback)
{
    QString message = QString("导航反馈 - %1").arg(feedback);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onNavigationPathReceived(const QVector<QPointF>& path)
{
    QString message = QString("导航路径 - 路径点数: %1").arg(path.size());
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    m_logTableModel->addLogEntry(entry);
}

// ==================== SystemMonitorThread槽函数 ====================

void MainWindow::onLogMessageReceived(const QString& message, int level, const QDateTime& timestamp)
{
    LogEntry entry(message, level, timestamp, "SystemMonitor", "ROS");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onCollisionDetected(const QString& message)
{
    QString logMessage = QString("碰撞检测 - %1").arg(message);
    LogEntry entry(logMessage, LOG_ERROR, QDateTime::currentDateTime(), "SystemMonitor", "Collision");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("警告: %1").arg(message), 5000);
}

void MainWindow::onAnomalyDetected(const QString& message)
{
    QString logMessage = QString("异常检测 - %1").arg(message);
    LogEntry entry(logMessage, LOG_WARNING, QDateTime::currentDateTime(), "SystemMonitor", "Anomaly");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("异常: %1").arg(message), 5000);
}

void MainWindow::onBehaviorTreeLogReceived(const QString& log)
{
    QString message = QString("行为树日志 - %1").arg(log);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "SystemMonitor", "BehaviorTree");
    m_logTableModel->addLogEntry(entry);
}

// ==================== LogThread槽函数 ====================

void MainWindow::onLogFileChanged(const QString& filePath)
{
    QString message = QString("日志文件变更 - 新文件: %1").arg(filePath);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "LogThread", "File");
    m_logTableModel->addLogEntry(entry);
}

// ==================== 线程状态槽函数 ====================

void MainWindow::onConnectionStateChanged(bool connected)
{
    QString status = connected ? "已连接" : "已断开";
    ui->statusbar->showMessage(QString("ROS连接状态: %1").arg(status), 3000);

    QString message = QString("连接状态变更 - %1").arg(status);
    LogEntry entry(message, connected ? LOG_INFO : LOG_WARNING, QDateTime::currentDateTime(), "System", "Connection");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onThreadStarted(const QString& threadName)
{
    QString message = QString("线程启动 - %1").arg(threadName);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "System", "Thread");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onThreadStopped(const QString& threadName)
{
    QString message = QString("线程停止 - %1").arg(threadName);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "System", "Thread");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onThreadError(const QString& error)
{
    QString message = QString("线程错误 - %1").arg(error);
    LogEntry entry(message, LOG_ERROR, QDateTime::currentDateTime(), "System", "Thread");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("错误: %1").arg(error), 5000);

    QMessageBox::critical(this, "线程错误", QString("发生线程错误:\n\n%1\n\n请检查日志获取详细信息。").arg(error));
}

// ==================== 日志查询方法 ====================

void MainWindow::queryLogsAsync(const QDateTime& startTime,
                               const QDateTime& endTime,
                               int minLevel,
                               const QString& source,
                               const QString& keyword,
                               int limit,
                               int offset)
{
    if (!m_logThread || !m_logThread->getStorageEngine()) {
        qWarning() << "[MainWindow] LogThread or StorageEngine is null";
        return;
    }

    LogStorageEngine* engine = m_logThread->getStorageEngine();
    if (!engine->isInitialized()) {
        qWarning() << "[MainWindow] StorageEngine is not initialized";
        return;
    }

    LogQueryTask* task = new LogQueryTask(engine, startTime, endTime, minLevel, source, keyword, limit, offset);

    connect(task, &LogQueryTask::queryCompleted, this, &MainWindow::onQueryCompleted, Qt::QueuedConnection);
    connect(task, &LogQueryTask::queryFailed, this, &MainWindow::onQueryFailed, Qt::QueuedConnection);

    m_threadPool->start(task);

    qDebug() << "[MainWindow] Async query started";
}

void MainWindow::onQueryCompleted(const QVector<StorageLogEntry>& results)
{
    qDebug() << "[MainWindow] Query completed, received" << results.size() << "logs";

    QVector<LogEntry> entries;
    entries.reserve(results.size());

    for (const auto& storageEntry : results) {
        LogEntry entry;
        entry.message = storageEntry.message;
        entry.level = storageEntry.level;
        entry.timestamp = storageEntry.timestamp;
        entry.source = storageEntry.source;
        entry.category = storageEntry.category;
        entries.append(entry);
    }

    m_logTableModel->clearLogs();
    m_logTableModel->addLogEntries(entries);

    ui->statusbar->showMessage(QString("查询完成: 找到 %1 条日志").arg(results.size()), 3000);
}

void MainWindow::onQueryFailed(const QString& error)
{
    qWarning() << "[MainWindow] Query failed:" << error;

    QString message = QString("查询失败 - %1").arg(error);
    LogEntry entry(message, LOG_ERROR, QDateTime::currentDateTime(), "MainWindow", "Query");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("查询失败: %1").arg(error), 5000);

    QMessageBox::warning(this, "查询失败", QString("日志查询失败:\n\n%1\n\n请检查数据库连接和查询参数。").arg(error));
}

void MainWindow::onFilterChanged()
{
    QSet<int> levels;
    if (ui->debugCheckBox->isChecked()) levels.insert(LOG_DEBUG);
    if (ui->infoCheckBox->isChecked()) levels.insert(LOG_INFO);
    if (ui->warningCheckBox->isChecked()) levels.insert(LOG_WARNING);
    if (ui->errorCheckBox->isChecked()) levels.insert(LOG_ERROR);
    if (ui->fatalCheckBox->isChecked()) levels.insert(LOG_FATAL);

    m_logFilterProxyModel->setLogLevelFilter(levels);
    m_logFilterProxyModel->setKeywordFilter(ui->keywordLineEdit->text());
    m_logFilterProxyModel->setSourceFilter(ui->sourceLineEdit->text());
    m_logFilterProxyModel->setTimeRangeFilter(ui->startTimeEdit->dateTime(), ui->endTimeEdit->dateTime());
}

void MainWindow::onQueryButtonClicked()
{
    QDateTime startTime = ui->startTimeEdit->dateTime();
    QDateTime endTime = ui->endTimeEdit->dateTime();

    QSet<int> levels;
    if (ui->debugCheckBox->isChecked()) levels.insert(LOG_DEBUG);
    if (ui->infoCheckBox->isChecked()) levels.insert(LOG_INFO);
    if (ui->warningCheckBox->isChecked()) levels.insert(LOG_WARNING);
    if (ui->errorCheckBox->isChecked()) levels.insert(LOG_ERROR);
    if (ui->fatalCheckBox->isChecked()) levels.insert(LOG_FATAL);

    int minLevel = levels.isEmpty() ? -1 : *std::min_element(levels.begin(), levels.end());
    QString source = ui->sourceLineEdit->text();
    QString keyword = ui->keywordLineEdit->text();

    queryLogsAsync(startTime, endTime, minLevel, source, keyword);
}

void MainWindow::onClearFilterButtonClicked()
{
    ui->debugCheckBox->setChecked(false);
    ui->infoCheckBox->setChecked(false);
    ui->warningCheckBox->setChecked(false);
    ui->errorCheckBox->setChecked(false);
    ui->fatalCheckBox->setChecked(false);
    ui->keywordLineEdit->clear();
    ui->sourceLineEdit->clear();
    ui->startTimeEdit->setDateTime(QDateTime::currentDateTime().addDays(-1));
    ui->endTimeEdit->setDateTime(QDateTime::currentDateTime());

    m_logFilterProxyModel->clearAllFilters();
}

void MainWindow::onRefreshButtonClicked()
{
    queryLogsAsync(QDateTime(), QDateTime(), -1, QString(), QString());
}

// ==================== 地图相关槽函数 ====================

void MainWindow::onMapReceived(const QImage& mapImage, double resolution, double originX, double originY)
{
    if (m_mapWidget) {
        m_mapWidget->setMapImage(mapImage, resolution, originX, originY);
    }
}

void MainWindow::onMapClicked(double x, double y)
{
    QString message = QString("地图点击位置: (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Map", "Click");
    m_logTableModel->addLogEntry(entry);
}

void MainWindow::onMapConnectionStateChanged(bool connected)
{
    QString status = connected ? "已连接" : "已断开";
    QString message = QString("地图连接状态: %1").arg(status);
    LogEntry entry(message, connected ? LOG_INFO : LOG_WARNING, QDateTime::currentDateTime(), "Map", "Connection");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("地图: %1").arg(status), 3000);

    if (!connected) {
        qWarning() << "[MainWindow] 地图连接已断开，请检查ROS2环境";
        QMessageBox::warning(this, "地图连接断开", "地图连接已断开，请检查ROS2环境\n\n可能的原因：\n1. ROS2 节点未启动\n2. 地图服务器未运行\n3. 网络连接问题");
    }
}

void MainWindow::onLoadMapFromFile()
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "选择地图文件",
        "",
        "YAML Files (*.yaml *.yml)"
    );

    if (filePath.isEmpty()) {
        return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map;

    if (m_mapCache && m_mapCache->contains(filePath)) {
        map = m_mapCache->get(filePath);
        qDebug() << "[MainWindow] 从缓存加载地图:" << filePath;
    } else {
        map = MapLoader::loadFromFile(filePath);
        if (!map) {
            QMessageBox::warning(this, "错误", "无法加载地图文件");
            return;
        }

        if (m_mapCache) {
            m_mapCache->add(filePath, map);
        }
    }

    QImage mapImage = MapConverter::convertToImage(map);
    if (mapImage.isNull()) {
        QMessageBox::warning(this, "错误", "无法转换地图图像");
        return;
    }

    if (m_mapWidget) {
        m_mapWidget->setMapImage(
            mapImage,
            map->info.resolution,
            map->info.origin.position.x,
            map->info.origin.position.y
        );
    }

    QString message = QString("地图文件加载成功 - 文件: %1, 尺寸: %2x%3, 分辨率: %4")
        .arg(filePath)
        .arg(map->info.width)
        .arg(map->info.height)
        .arg(map->info.resolution, 0, 'f', 3);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Map", "FileLoad");
    m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("地图已加载: %1").arg(QFileInfo(filePath).fileName()), 3000);
}
