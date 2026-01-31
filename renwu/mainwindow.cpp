#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QFileDialog>
#include <QTimer>
#include <QCoreApplication>
#include <algorithm>
#include <QStandardPaths>
#include "maploader.h"
#include "mapconverter.h"
#include "mapmarker.h"
#include "roscontextmanager.h"
#include <nav_msgs/msg/path.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_targetX(0.0)
    , m_targetY(0.0)
    , m_targetYaw(0.0)
    , m_hasTarget(false)
    , m_startTime(QDateTime::currentDateTime())
    , m_initialDistance(0.0)
{
    ui->setupUi(this);
}

bool MainWindow::initialize()
{
    // 使用智能指针管理资源，自动释放
    m_userStorageEngine = std::make_unique<UserStorageEngine>(this);
    if (!m_userStorageEngine->initialize()) {
        qCritical() << "[MainWindow] 错误：用户存储引擎初始化失败！";
        QMessageBox::critical(this, "错误", "用户存储引擎初始化失败！\n\n程序无法继续运行。");
        return false;
    }

    m_userAuthManager = std::make_unique<UserAuthManager>(this);
    if (!m_userAuthManager->initialize(m_userStorageEngine.get())) {
        qCritical() << "[MainWindow] 错误：用户认证管理器初始化失败！";
        QMessageBox::critical(this, "错误", "用户认证管理器初始化失败！\n\n程序无法继续运行。");
        return false;
    }

    m_loginDialog = std::make_unique<LoginDialog>(m_userAuthManager.get(), this);

    m_logTableModel = std::make_unique<LogTableModel>(this);
    m_logTableModel->setBatchUpdateEnabled(true);
    m_logTableModel->setBatchUpdateInterval(200);
    m_logFilterProxyModel = std::make_unique<LogFilterProxyModel>(this);
    m_logFilterProxyModel->setSourceModel(m_logTableModel.get());
    ui->logTableView->setModel(m_logFilterProxyModel.get());

    ui->logTableView->setColumnWidth(0, 180);
    ui->logTableView->setColumnWidth(1, 80);
    ui->logTableView->setColumnWidth(2, 120);
    ui->logTableView->horizontalHeader()->setStretchLastSection(true);

    // 初始化 ROS2 上下文
    ROSContextManager::instance().initialize();

    // 创建 Nav2ViewWidget，从ROS参数获取地图路径
    auto node = std::make_shared<rclcpp::Node>("nav2_view_node");
    std::string map_path = getMapPathFromRosParam(node).toStdString();
    m_nav2ViewWidget = std::make_unique<Nav2ViewWidget>(map_path, node, this);

    // 连接地图加载信号，处理加载失败情况
    connect(m_nav2ViewWidget.get(), &Nav2ViewWidget::mapLoadFailed,
            this, [this](const QString& error) {
                qWarning() << "[MainWindow]" << error;
                // 地图加载失败时显示警告但不阻塞程序
                // 用户可以通过界面功能选择其他地图
            });

    ui->nav2MapLayout->addWidget(m_nav2ViewWidget.get());

    m_mapCache = std::make_unique<MapCache>(10, this);

    m_navigationClient = std::make_unique<NavigationActionClient>(this);
    m_pathVisualizer = nullptr;  // Nav2ViewWidget 内部处理路径显示

    QTimer* currentTimeTimer = new QTimer(this);
    connect(currentTimeTimer, &QTimer::timeout, this, &MainWindow::updateCurrentTime);
    currentTimeTimer->start(1000);

    initializeThreads();
    connectSignals();
    startAllThreads();

    // 尝试从环境变量自动登录（仅用于测试）
    QString auto_username = qgetenv("WHEELTEC_USERNAME");
    QString auto_password = qgetenv("WHEELTEC_PASSWORD");
    if (!auto_username.isEmpty() && !auto_password.isEmpty()) {
        qDebug() << "[MainWindow] 尝试自动登录...";
        if (m_userAuthManager->login(auto_username, auto_password)) {
            qDebug() << "[MainWindow] 自动登录成功:" << m_userAuthManager->getCurrentUsername();
        } else {
            qWarning() << "[MainWindow] 自动登录失败:" << m_userAuthManager->getLastError();
            // 自动登录失败不阻塞，让用户手动登录
        }
    }

    return true;
}

MainWindow::~MainWindow()
{
    stopAllThreads();

    // 智能指针自动管理资源，无需手动delete
    delete ui;
}

void MainWindow::initializeThreads()
{
    // 创建统一的日志存储引擎
    m_logStorage = std::make_unique<LogStorageEngine>();
    QString dbPath = QCoreApplication::applicationDirPath() + "/logs/unified_logs.db";
    if (!m_logStorage->initialize(dbPath)) {
        qWarning() << "[MainWindow] Failed to initialize LogStorageEngine:" << m_logStorage->getLastError();
    }

    m_robotStatusThread = std::make_unique<RobotStatusThread>(this);
    m_navStatusThread = std::make_unique<NavStatusThread>(this);
    m_systemMonitorThread = std::make_unique<SystemMonitorThread>(m_logStorage.get(), this);
    m_logThread = std::make_unique<LogThread>(m_logStorage.get(), this);
    m_paramThread = std::make_unique<Nav2ParameterThread>(this);

    m_logTableModel->setStorageEngine(m_logThread->getStorageEngine());
}

void MainWindow::connectSignals()
{
    // RobotStatusThread信号连接
    connect(m_robotStatusThread.get(), &RobotStatusThread::batteryStatusReceived,
            this, &MainWindow::onBatteryStatusReceived);
    connect(m_robotStatusThread.get(), &RobotStatusThread::positionReceived,
            this, &MainWindow::onPositionReceived);
    connect(m_robotStatusThread.get(), &RobotStatusThread::odometryReceived,
            this, &MainWindow::onOdometryReceived);
    connect(m_robotStatusThread.get(), &RobotStatusThread::systemTimeReceived,
            this, &MainWindow::onSystemTimeReceived);
    connect(m_robotStatusThread.get(), &RobotStatusThread::diagnosticsReceived,
            this, &MainWindow::onDiagnosticsReceived);
    connect(m_robotStatusThread.get(), &RobotStatusThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // RobotStatusThread日志转发给LogThread
    connect(m_robotStatusThread.get(), &RobotStatusThread::logMessage,
            m_logThread.get(), &LogThread::writeLog);

    // NavStatusThread信号连接
    connect(m_navStatusThread.get(), &NavStatusThread::navigationStatusReceived,
            this, &MainWindow::onNavigationStatusReceived);
    connect(m_navStatusThread.get(), &NavStatusThread::navigationPathReceived,
            this, &MainWindow::onNavigationPathReceived);
    connect(m_navStatusThread.get(), &NavStatusThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // NavStatusThread日志转发给LogThread
    connect(m_navStatusThread.get(), &NavStatusThread::logMessage,
            m_logThread.get(), &LogThread::writeLog);

    // SystemMonitorThread信号连接
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::logMessageReceived,
            this, &MainWindow::onLogMessageReceived);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::collisionDetected,
            this, &MainWindow::onCollisionDetected);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::anomalyDetected,
            this, &MainWindow::onAnomalyDetected);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::behaviorTreeLogReceived,
            this, &MainWindow::onBehaviorTreeLogReceived);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);

    // SystemMonitorThread日志转发给LogThread
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::logMessage,
            m_logThread.get(), &LogThread::writeLog);

    // SystemMonitorThread日志接收后转发给LogThread
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::logMessageReceived,
            m_logThread.get(), [this](const QString& message, int level, const QDateTime& timestamp) {
        LogEntry entry(message, static_cast<LogLevel>(level), timestamp, "SystemMonitor", "ROS");
        m_logThread->writeLogEntry(entry);
    });

    // RobotStatusThread的诊断信息转发给SystemMonitorThread
    connect(m_robotStatusThread.get(), &RobotStatusThread::diagnosticsReceived,
            m_systemMonitorThread.get(), &SystemMonitorThread::onDiagnosticsReceived);

    // LogThread信号连接
    connect(m_logThread.get(), &LogThread::logFileChanged,
            this, &MainWindow::onLogFileChanged);

    // 线程状态信号连接
    connect(m_robotStatusThread.get(), &RobotStatusThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_robotStatusThread.get(), &RobotStatusThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_robotStatusThread.get(), &RobotStatusThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_navStatusThread.get(), &NavStatusThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_navStatusThread.get(), &NavStatusThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_navStatusThread.get(), &NavStatusThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_systemMonitorThread.get(), &SystemMonitorThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_systemMonitorThread.get(), &SystemMonitorThread::threadError,
            this, &MainWindow::onThreadError);

    connect(m_logThread.get(), &LogThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_logThread.get(), &LogThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_logThread.get(), &LogThread::threadError,
            this, &MainWindow::onThreadError);

    // 连接过滤控件信号
    connect(ui->debugCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->infoCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->warningCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->errorCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);
    connect(ui->fatalCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onFilterChanged);

    // 连接按钮信号
    connect(ui->btnStartNavigation, &QPushButton::clicked, this, &MainWindow::onStartNavigation);
    connect(ui->btnCancelNavigation, &QPushButton::clicked, this, &MainWindow::onCancelNavigation);
    connect(ui->btnClearGoal, &QPushButton::clicked, this, &MainWindow::onClearGoal);

    // 用户菜单信号连接
    connect(ui->actionUserManagement, &QAction::triggered, this, &MainWindow::onUserManagement);
    connect(ui->actionChangePassword, &QAction::triggered, this, &MainWindow::onChangePassword);
    connect(ui->actionLogout, &QAction::triggered, this, &MainWindow::onLogout);

    // NavigationActionClient信号连接
    connect(m_navigationClient.get(), &NavigationActionClient::goalAccepted,
            this, &MainWindow::onGoalAccepted);
    connect(m_navigationClient.get(), &NavigationActionClient::goalRejected,
            this, &MainWindow::onGoalRejected);
    connect(m_navigationClient.get(), &NavigationActionClient::feedbackReceived,
            this, &MainWindow::onNavigationFeedback);
    connect(m_navigationClient.get(), &NavigationActionClient::resultReceived,
            this, &MainWindow::onNavigationResult);
    connect(m_navigationClient.get(), &NavigationActionClient::goalCanceled,
            this, &MainWindow::onGoalCanceled);

    // Nav2ViewWidget信号连接
    connect(m_nav2ViewWidget.get(), &Nav2ViewWidget::goalPosePreview,
            this, [this](double x, double y, double yaw) {
                m_targetX = x;
                m_targetY = y;
                m_targetYaw = yaw;
                m_hasTarget = true;
                ui->labelTargetPosition->setText(QString("(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
            });

    // Nav2ParameterThread 信号连接
    connect(m_paramThread.get(), &Nav2ParameterThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_paramThread.get(), &Nav2ParameterThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_paramThread.get(), &Nav2ParameterThread::threadError,
            this, &MainWindow::onThreadError);
    connect(m_paramThread.get(), &Nav2ParameterThread::parameterRefreshed,
            this, &MainWindow::onParameterRefreshed);
    connect(m_paramThread.get(), &Nav2ParameterThread::parameterApplied,
            this, &MainWindow::onParameterApplied);
    connect(m_paramThread.get(), &Nav2ParameterThread::operationFinished,
            this, &MainWindow::onParameterOperationFinished);

    // Settings Tab 按钮连接
    connect(ui->refreshButton, &QPushButton::clicked, this, &MainWindow::onRefreshButtonClicked);
    connect(ui->applyButton, &QPushButton::clicked, this, &MainWindow::onApplyButtonClicked);
    connect(ui->resetButton, &QPushButton::clicked, this, &MainWindow::onResetButtonClicked);
    connect(ui->discardButton, &QPushButton::clicked, this, &MainWindow::onDiscardButtonClicked);

    // 参数控件值变化连接
    connect(ui->maxVelXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->minVelXSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->maxVelThetaSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->lookaheadDistSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->localInflationRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->localCostScalingFactorSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->globalInflationRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->globalCostScalingFactorSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->smootherMaxVelSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->smootherMinVelSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->smootherMaxAccelSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->smootherMaxDecelSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
    connect(ui->robotRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &MainWindow::onParameterValueChanged);
}

void MainWindow::startAllThreads()
{
    // 使用信号槽机制确保线程按顺序启动，替代硬编码延迟
    // 启动顺序：日志 -> 系统监控 -> 机器人状态 -> 导航状态 -> 地图 -> 参数

    // 连接线程启动完成信号到下一个线程的启动槽
    connect(m_logThread.get(), &LogThread::threadStarted, this, [this]() {
        QTimer::singleShot(THREAD_START_DELAY_MS, m_systemMonitorThread.get(), [this]() {
            m_systemMonitorThread->start();
        });
    }, Qt::UniqueConnection);

    connect(m_systemMonitorThread.get(), &SystemMonitorThread::threadStarted, this, [this]() {
        QTimer::singleShot(THREAD_START_DELAY_MS, m_robotStatusThread.get(), [this]() {
            m_robotStatusThread->start();
        });
    }, Qt::UniqueConnection);

    connect(m_robotStatusThread.get(), &RobotStatusThread::threadStarted, this, [this]() {
        QTimer::singleShot(THREAD_START_DELAY_MS, m_navStatusThread.get(), [this]() {
            m_navStatusThread->start();
        });
    }, Qt::UniqueConnection);

    connect(m_navStatusThread.get(), &NavStatusThread::threadStarted, this, [this]() {
        QTimer::singleShot(THREAD_START_DELAY_MS, m_paramThread.get(), [this]() {
            m_paramThread->start();
        });
    }, Qt::UniqueConnection);

    // 启动第一个线程
    m_logThread->start();
}

void MainWindow::stopAllThreads()
{
    // 停止顺序与启动相反
    // 使用智能指针的bool转换操作符检查有效性
    if (m_paramThread && m_paramThread->isRunning()) {
        m_paramThread->stopThread();
        if (!m_paramThread->wait(THREAD_STOP_TIMEOUT_MS)) {
            qWarning() << "[MainWindow] 警告：参数线程停止超时";
        }
    }

    if (m_navStatusThread && m_navStatusThread->isRunning()) {
        m_navStatusThread->stopThread();
        if (!m_navStatusThread->wait(THREAD_STOP_TIMEOUT_MS)) {
            qWarning() << "[MainWindow] 警告：导航状态线程停止超时";
        }
    }

    if (m_robotStatusThread && m_robotStatusThread->isRunning()) {
        m_robotStatusThread->stopThread();
        if (!m_robotStatusThread->wait(THREAD_STOP_TIMEOUT_MS)) {
            qWarning() << "[MainWindow] 警告：机器人状态线程停止超时";
        }
    }

    if (m_systemMonitorThread && m_systemMonitorThread->isRunning()) {
        m_systemMonitorThread->stopThread();
        if (!m_systemMonitorThread->wait(THREAD_STOP_TIMEOUT_MS)) {
            qWarning() << "[MainWindow] 警告：系统监控线程停止超时";
        }
    }

    if (m_logThread && m_logThread->isRunning()) {
        m_logThread->stopThread();
        if (!m_logThread->wait(THREAD_STOP_TIMEOUT_MS)) {
            qWarning() << "[MainWindow] 警告：日志线程停止超时";
        }
    }
}

// ==================== RobotStatusThread槽函数 ====================

void MainWindow::onBatteryStatusReceived(float voltage, float percentage)
{
    ui->labelVoltage->setText(QString("%1 V").arg(voltage, 0, 'f', 2));
    ui->labelPercentage->setText(QString("%1 %").arg(percentage, 0, 'f', 1));

    QString statusText;
    QString color;

    if (voltage > 13.0f) {
        statusText = "充电中";
        color = "green";
    } else if (percentage < 20.0f) {
        statusText = "低电量警告";
        color = "red";
    } else {
        statusText = "放电中";
        color = "blue";
    }

    ui->labelBatteryStatus->setText(QString("状态：%1").arg(statusText));
    ui->labelBatteryStatus->setStyleSheet(QString("color: %1;").arg(color));

    if (percentage < 20.0f) {
        ui->labelPercentage->setStyleSheet("color: red; font-weight: bold;");
    } else {
        ui->labelPercentage->setStyleSheet("");
    }

    QString message = QString("电池状态 - 电压: %1 V, 电量: %2%").arg(voltage, 0, 'f', 2).arg(percentage, 0, 'f', 1);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "RobotStatus", "Battery");
    addLogEntry(entry);
}

void MainWindow::onPositionReceived(double x, double y, double yaw)
{
    ui->labelX->setText(QString("%1 m").arg(x, 0, 'f', 3));
    ui->labelY->setText(QString("%1 m").arg(y, 0, 'f', 3));
    
    double yaw_degrees = yaw * 180.0 / M_PI;
    ui->labelYaw->setText(QString("%1 °").arg(yaw_degrees, 0, 'f', 1));

    QString message = QString("位置信息 - X: %1, Y: %2, Yaw: %3").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "RobotStatus", "Position");
    addLogEntry(entry);
}

void MainWindow::onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega)
{
    if (ui->labelX->text() == "-- m") {
        ui->labelX->setText(QString("%1 m").arg(x, 0, 'f', 3));
        ui->labelY->setText(QString("%1 m").arg(y, 0, 'f', 3));

        double yaw_degrees = yaw * 180.0 / M_PI;
        ui->labelYaw->setText(QString("%1 °").arg(yaw_degrees, 0, 'f', 1));
    }

    // 更新速度显示
    ui->labelLinearVel->setText(QString("%1 m/s").arg(vx, 0, 'f', 2));
    ui->labelAngularVel->setText(QString("%1 rad/s").arg(omega, 0, 'f', 2));

    QString message = QString("里程计 - 位置(X:%1, Y:%2, Yaw:%3), 速度(vx:%4, vy:%5, omega:%6)")
        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2)
        .arg(vx, 0, 'f', 2).arg(vy, 0, 'f', 2).arg(omega, 0, 'f', 2);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "RobotStatus", "Odometry");
    addLogEntry(entry);
}

void MainWindow::onSystemTimeReceived(const QString& time)
{
    ui->labelCurrentTime->setText(time);
    ui->statusbar->showMessage(QString("系统时间: %1").arg(time));
}

void MainWindow::onDiagnosticsReceived(const QString& status, int level, const QString& message)
{
    LogLevel logLevel = LogLevel::INFO;
    if (level == 1) logLevel = LogLevel::WARNING;
    else if (level == 2) logLevel = LogLevel::ERROR;
    else if (level == 3) logLevel = LogLevel::FATAL;
    else if (level == 4) logLevel = LogLevel::DEBUG;

    QString logMessage = QString("诊断信息 - 状态: %1, 消息: %2").arg(status).arg(message);
    LogEntry entry(logMessage, logLevel, QDateTime::currentDateTime(), "RobotStatus", "Diagnostics");
    addLogEntry(entry);

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
    LogEntry entry(logMessage, LogLevel::INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    addLogEntry(entry);

    ui->statusbar->showMessage(QString("导航: %1 - %2").arg(statusStr).arg(message), 3000);
}

void MainWindow::onNavigationPathReceived(const QVector<QPointF>& path)
{
    QString message = QString("导航路径 - 路径点数: %1").arg(path.size());
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    addLogEntry(entry);

    // Nav2ViewWidget 内部订阅 /plan 话题处理路径显示
}

// ==================== SystemMonitorThread槽函数 ====================

void MainWindow::onLogMessageReceived(const QString& message, int level, const QDateTime& timestamp)
{
    LogEntry entry(message, static_cast<LogLevel>(level), timestamp, "SystemMonitor", "ROS");
    addLogEntry(entry);
}

void MainWindow::onCollisionDetected(const QString& message)
{
    QString logMessage = QString("碰撞检测 - %1").arg(message);
    LogEntry entry(logMessage, LogLevel::ERROR, QDateTime::currentDateTime(), "SystemMonitor", "Collision");
    addLogEntry(entry);

    ui->statusbar->showMessage(QString("警告: %1").arg(message), 5000);
}

void MainWindow::onAnomalyDetected(const QString& message)
{
    QString logMessage = QString("异常检测 - %1").arg(message);
    LogEntry entry(logMessage, LogLevel::WARNING, QDateTime::currentDateTime(), "SystemMonitor", "Anomaly");
    addLogEntry(entry);

    ui->statusbar->showMessage(QString("异常: %1").arg(message), 5000);
}

void MainWindow::onBehaviorTreeLogReceived(const QString& log)
{
    QString message = QString("行为树日志 - %1").arg(log);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "SystemMonitor", "BehaviorTree");
    addLogEntry(entry);
}

// ==================== LogThread槽函数 ====================

void MainWindow::onLogFileChanged(const QString& filePath)
{
    QString message = QString("日志文件变更 - 新文件: %1").arg(filePath);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "LogThread", "File");
    addLogEntry(entry);
}

// ==================== 线程状态槽函数 ====================

void MainWindow::onConnectionStateChanged(bool connected)
{
    QString status = connected ? "已连接" : "已断开";
    ui->statusbar->showMessage(QString("ROS连接状态: %1").arg(status), 3000);

    QString message = QString("连接状态变更 - %1").arg(status);
    LogLevel logLevel = connected ? LogLevel::INFO : LogLevel::WARNING;
    LogEntry entry(message, logLevel, QDateTime::currentDateTime(), "System", "Connection");
    addLogEntry(entry);
}

void MainWindow::onThreadStarted(const QString& threadName)
{
    QString message = QString("线程启动 - %1").arg(threadName);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "System", "Thread");
    addLogEntry(entry);
}

void MainWindow::onThreadStopped(const QString& threadName)
{
    QString message = QString("线程停止 - %1").arg(threadName);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "System", "Thread");
    addLogEntry(entry);
}

void MainWindow::onThreadError(const QString& error)
{
    QString message = QString("线程错误 - %1").arg(error);
    // LogEntry entry(message, LOG_ERROR, QDateTime::currentDateTime(), "System", "Thread");
    // m_logTableModel->addLogEntry(entry);

    ui->statusbar->showMessage(QString("错误: %1").arg(error), 5000);

    QMessageBox::critical(this, "线程错误", QString("发生线程错误:\n\n%1\n\n请检查日志获取详细信息。").arg(error));
}

void MainWindow::onFilterChanged()
{
    refreshLogDisplay(false);  // 手动过滤不自动滚动
}

void MainWindow::refreshLogDisplay(bool autoScroll)
{
    // 获取当前勾选的日志级别
    QSet<int> enabledLevels;
    if (ui->debugCheckBox->isChecked()) enabledLevels << static_cast<int>(LogLevel::DEBUG);
    if (ui->infoCheckBox->isChecked()) enabledLevels << static_cast<int>(LogLevel::INFO);
    if (ui->warningCheckBox->isChecked()) enabledLevels << static_cast<int>(LogLevel::WARNING);
    if (ui->errorCheckBox->isChecked()) enabledLevels << static_cast<int>(LogLevel::ERROR);
    if (ui->fatalCheckBox->isChecked()) enabledLevels << static_cast<int>(LogLevel::FATAL);

    // 使用LogFilterProxyModel的过滤功能，避免重建模型
    m_logFilterProxyModel->setLogLevelFilter(enabledLevels);

    if (autoScroll) {
        ui->logTableView->scrollToBottom();
    }
}

bool MainWindow::shouldDisplayLog(int level) const
{
    switch (static_cast<LogLevel>(level)) {
        case LogLevel::DEBUG:   return ui->debugCheckBox->isChecked();
        case LogLevel::INFO:    return ui->infoCheckBox->isChecked();
        case LogLevel::WARNING: return ui->warningCheckBox->isChecked();
        case LogLevel::ERROR:   return ui->errorCheckBox->isChecked();
        case LogLevel::FATAL:   return ui->fatalCheckBox->isChecked();
        default:                return false;
    }
}

// ==================== 地图相关槽函数 ====================

void MainWindow::onMapClicked(double x, double y)
{
    // Nav2ViewWidget 通过鼠标拖拽设置目标，不需要此函数
    m_targetX = x;
    m_targetY = y;
    m_targetYaw = 0.0;
    m_hasTarget = true;

    // if (m_mapWidget) {
    //     m_mapWidget->clearMarkers();
    //     MapMarker marker(x, y, QColor(255, 0, 0), "目标点", "导航目标");
    //     m_mapWidget->addMarker(marker);
    // }

    ui->labelTargetPosition->setText(QString("(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));

    QString message = QString("地图点击位置: (%1, %2) - 已设置为导航目标").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "Map", "Click");
    addLogEntry(entry);
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

    // 重置旧的 Nav2ViewWidget
    m_nav2ViewWidget.reset();

    // 创建新的 Nav2ViewWidget 加载新地图
    auto node = std::make_shared<rclcpp::Node>("nav2_view_node");
    m_nav2ViewWidget = std::make_unique<Nav2ViewWidget>(filePath.toStdString(), node, this);
    ui->nav2MapLayout->addWidget(m_nav2ViewWidget.get());

    QString message = QString("地图文件加载成功 - 文件: %1").arg(filePath);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "Map", "FileLoad");
    addLogEntry(entry);

    ui->statusbar->showMessage(QString("地图已加载: %1").arg(QFileInfo(filePath).fileName()), 3000);
}

void MainWindow::onStartNavigation()
{
    if (!m_hasTarget) {
        QMessageBox::warning(this, tr("提示"), tr("请设置导航目标"));
        return;
    }

    // 检查指针有效性
    if (!m_nav2ViewWidget) {
        qCritical() << "[MainWindow] 错误：Nav2ViewWidget 未初始化";
        QMessageBox::critical(this, tr("错误"), tr("导航视图组件未初始化"));
        return;
    }

    // 调用 Nav2ViewWidget 发布目标到 /goal_pose 话题
    m_nav2ViewWidget->publishCurrentGoal();

    ui->labelNavigationStatus->setText(tr("导航中..."));
    ui->labelNavigationStatus->setStyleSheet("color: blue;");
}

void MainWindow::onCancelNavigation()
{
    // 检查指针有效性
    if (!m_navigationClient) {
        qCritical() << "[MainWindow] 错误：NavigationActionClient 未初始化";
        QMessageBox::critical(this, tr("错误"), tr("导航客户端未初始化"));
        return;
    }

    if (!m_navigationClient->isNavigating()) {
        QMessageBox::information(this, tr("提示"), tr("无导航任务"));
        return;
    }

    bool success = m_navigationClient->cancelGoal();
    if (!success) {
        QMessageBox::warning(this, tr("警告"), tr("取消导航失败"));
        return;
    }

    if (m_nav2ViewWidget) {
        m_nav2ViewWidget->clearGoal();
    }

    ui->labelNavigationStatus->setText(tr("已取消"));
    ui->labelNavigationStatus->setStyleSheet("color: orange;");
}

void MainWindow::onClearGoal()
{
    m_hasTarget = false;
    m_targetX = 0.0;
    m_targetY = 0.0;
    m_targetYaw = 0.0;
    m_initialDistance = 0.0;

    // 清除 Nav2ViewWidget 的目标状态，检查指针有效性
    if (m_nav2ViewWidget) {
        m_nav2ViewWidget->clearGoal();
    }

    ui->labelTargetPosition->setText(tr("未设置"));
    ui->labelNavigationStatus->setText(tr("空闲"));
    ui->labelNavigationStatus->setStyleSheet("color: gray;");
    ui->labelDistanceRemaining->setText("-- m");
    ui->labelNavigationTime->setText("-- s");
    ui->labelRecoveries->setText("0");
    ui->navigationProgressBar->setValue(0);
    ui->labelEstimatedTime->setText("--:--");
}

void MainWindow::onNavigationFeedback(double distanceRemaining, double navigationTime, int recoveries, double estimatedTimeRemaining)
{
    ui->labelDistanceRemaining->setText(QString::number(distanceRemaining, 'f', 2) + " m");
    ui->labelNavigationTime->setText(QString::number(navigationTime, 'f', 1) + " s");
    ui->labelRecoveries->setText(QString::number(recoveries));

    // 第一次收到反馈时记录初始距离
    if (m_initialDistance <= 0.0 && distanceRemaining > 0.0) {
        m_initialDistance = distanceRemaining;
    }

    // 计算导航进度
    if (m_initialDistance > 0.0 && distanceRemaining >= 0.0) {
        int progress = static_cast<int>(((m_initialDistance - distanceRemaining) / m_initialDistance) * 100.0);
        progress = std::max(0, std::min(100, progress));
        ui->navigationProgressBar->setValue(progress);
    }

    // 格式化预计剩余时间为 MM:SS 格式
    int minutes = static_cast<int>(estimatedTimeRemaining) / 60;
    int seconds = static_cast<int>(estimatedTimeRemaining) % 60;
    QString timeStr = QString("%1:%2")
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));
    ui->labelEstimatedTime->setText(timeStr);

    QString message = QString("导航反馈 - 剩余距离: %1m, 导航时间: %2s, 恢复次数: %3, 预计剩余: %4")
        .arg(distanceRemaining, 0, 'f', 2)
        .arg(navigationTime, 0, 'f', 1)
        .arg(recoveries)
        .arg(timeStr);
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "Navigation", "Feedback");
    addLogEntry(entry);
}

void MainWindow::onNavigationResult(bool success, const QString& message)
{
    // 检查指针有效性
    if (!m_navigationClient) {
        qCritical() << "[MainWindow] 错误：NavigationActionClient 未初始化";
        return;
    }

    if (success) {
        QMessageBox::information(this, tr("导航完成"), tr("导航成功到达目标点"));
        ui->labelNavigationStatus->setText(tr("导航成功"));
        ui->labelNavigationStatus->setStyleSheet("color: green;");
        ui->navigationProgressBar->setValue(100);
    } else {
        QMessageBox::warning(this, tr("导航失败"), message);
        ui->labelNavigationStatus->setText(tr("导航失败"));
        ui->labelNavigationStatus->setStyleSheet("color: red;");
    }

    // 重置初始距离
    m_initialDistance = 0.0;

    QString logMessage = QString("导航结果 - %1: %2").arg(success ? tr("成功") : tr("失败")).arg(message);
    LogLevel logLevel = success ? LogLevel::INFO : LogLevel::WARNING;
    LogEntry entry(logMessage, logLevel, QDateTime::currentDateTime(), "Navigation", "Result");
    addLogEntry(entry);
}

void MainWindow::onGoalAccepted()
{
    ui->labelNavigationStatus->setText("导航中...");
    ui->labelNavigationStatus->setStyleSheet("color: blue;");

    // 重置初始距离，将在第一次反馈时设置
    m_initialDistance = 0.0;

    QString message = "导航目标已被接受，开始导航";
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "Navigation", "Goal");
    addLogEntry(entry);
    ui->statusbar->showMessage(message, 3000);
}

void MainWindow::onGoalRejected(const QString& reason)
{
    QMessageBox::warning(this, "目标被拒绝", reason);
    ui->labelNavigationStatus->setText("目标被拒绝");
    ui->labelNavigationStatus->setStyleSheet("color: orange;");

    QString message = QString("导航目标被拒绝 - 原因: %1").arg(reason);
    LogEntry entry(message, LogLevel::WARNING, QDateTime::currentDateTime(), "Navigation", "Goal");
    addLogEntry(entry);
}

void MainWindow::onGoalCanceled()
{
    QMessageBox::information(this, "导航已取消", "导航目标已取消");
    ui->labelNavigationStatus->setText("已取消");
    ui->labelNavigationStatus->setStyleSheet("color: gray;");

    QString message = "导航目标已取消";
    LogEntry entry(message, LogLevel::INFO, QDateTime::currentDateTime(), "Navigation", "Goal");
    addLogEntry(entry);
}

void MainWindow::updateCurrentTime()
{
    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->labelCurrentTime->setText(currentTime);

    qint64 uptimeSec = m_startTime.secsTo(QDateTime::currentDateTime());
    int hours = uptimeSec / 3600;
    int minutes = (uptimeSec % 3600) / 60;
    int seconds = uptimeSec % 60;

    QString uptimeStr = QString("%1:%2:%3")
        .arg(hours, 2, 10, QChar('0'))
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));

    ui->labelUptime->setText(uptimeStr);
}

// ==================== 用户权限管理槽函数 ====================

void MainWindow::onLoginSuccess(const User& user)
{
    updateUIBasedOnPermission();
}

void MainWindow::onLoginFailed(const QString& reason)
{
}

void MainWindow::onLogout()
{
    try {
        // 检查指针有效性
        if (!m_userAuthManager) {
            qCritical() << "[MainWindow] 错误：UserAuthManager 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("用户认证管理器未初始化"));
            return;
        }

        if (!m_loginDialog) {
            qCritical() << "[MainWindow] 错误：LoginDialog 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("登录对话框未初始化"));
            return;
        }

        // 检查是否有正在进行的导航操作
        if (m_hasTarget && m_navigationClient && m_navigationClient->isNavigating()) {
            QMessageBox::StandardButton reply = QMessageBox::question(
                this,
                tr("确认登出"),
                tr("导航正在进行中，确定要登出吗？"),
                QMessageBox::Yes | QMessageBox::No
            );
            if (reply == QMessageBox::No) {
                return;
            }
            // 取消当前导航目标
            onClearGoal();
        }

        m_userAuthManager->logout();

        // 清理当前状态
        m_hasTarget = false;
        m_targetX = 0.0;
        m_targetY = 0.0;
        m_targetYaw = 0.0;

        // 重新登录循环，添加最大重试次数限制
        int maxRetries = 3;
        int retryCount = 0;
        
        while (retryCount < maxRetries) {
            m_loginDialog->exec();

            if (m_userAuthManager->isLoggedIn()) {
                break;
            }

            retryCount++;
            
            if (retryCount >= maxRetries) {
                qCritical() << "[MainWindow] 错误：登录重试次数超过限制，程序将退出";
                QMessageBox::critical(this, tr("登录失败"), 
                    tr("登录失败次数过多，程序将退出"));
                QApplication::quit();
                return;
            }

            QMessageBox::StandardButton reply = QMessageBox::question(
                this,
                tr("登录失败"),
                tr("登录失败，是否重试？（剩余 %1 次）").arg(maxRetries - retryCount),
                QMessageBox::Retry | QMessageBox::Cancel
            );

            if (reply == QMessageBox::Cancel) {
                this->close();
                return;
            }
        }

        updateUIBasedOnPermission();
        
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onLogout 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), 
            tr("登出过程中发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onLogout 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("登出过程中发生未知错误"));
    }
}

void MainWindow::onUserManagement()
{
    try {
        // 检查指针有效性
        if (!m_userAuthManager) {
            qCritical() << "[MainWindow] 错误：UserAuthManager 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("用户认证管理器未初始化"));
            return;
        }

        if (!m_userAuthManager->canAdmin()) {
            QMessageBox::warning(this, tr("权限不足"), tr("您没有权限访问用户管理功能"));
            return;
        }

        UserManagementDialog dialog(m_userAuthManager.get(), this);
        dialog.exec();
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onUserManagement 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), 
            tr("打开用户管理对话框时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onUserManagement 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("打开用户管理对话框时发生未知错误"));
    }
}

void MainWindow::onChangePassword()
{
    try {
        // 检查指针有效性
        if (!m_userAuthManager) {
            qCritical() << "[MainWindow] 错误：UserAuthManager 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("用户认证管理器未初始化"));
            return;
        }

        ChangePasswordDialog dialog(ChangePasswordDialog::Mode::SelfChange, m_userAuthManager.get(), "", this);
        dialog.exec();
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onChangePassword 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), 
            tr("打开修改密码对话框时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onChangePassword 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("打开修改密码对话框时发生未知错误"));
    }
}

void MainWindow::updateUIBasedOnPermission()
{
    try {
        // 检查指针有效性
        if (!m_userAuthManager) {
            qCritical() << "[MainWindow] 错误：UserAuthManager 未初始化";
            return;
        }

        UserPermission permission = m_userAuthManager->getCurrentPermission();

        bool canOperate = m_userAuthManager->canOperate();
        bool canAdmin = m_userAuthManager->canAdmin();

        ui->btnStartNavigation->setEnabled(canOperate);
        ui->btnCancelNavigation->setEnabled(canOperate);
        ui->btnClearGoal->setEnabled(canOperate);

        QString permissionText;
        switch (permission) {
            case UserPermission::VIEWER:
                permissionText = tr("查看者");
                break;
            case UserPermission::OPERATOR:
                permissionText = tr("操作员");
                break;
            case UserPermission::ADMIN:
                permissionText = tr("管理员");
                break;
        }

        QString username = m_userAuthManager->getCurrentUsername();
        if (username.isEmpty()) {
            username = tr("未登录");
        }
        
        QString statusText = QString(tr("当前用户: %1 (%2)")).arg(username).arg(permissionText);
        ui->statusbar->showMessage(statusText, 0);

        qDebug() << "[MainWindow] UI updated based on permission:" << permissionText;
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] updateUIBasedOnPermission 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] updateUIBasedOnPermission 发生未知异常";
    }
}

// ==================== Nav2ParameterThread 槽函数 ====================

void MainWindow::onRefreshButtonClicked()
{
    try {
        // 检查指针有效性
        if (!m_paramThread) {
            qCritical() << "[MainWindow] 错误：Nav2ParameterThread 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("参数线程未初始化"));
            return;
        }

        m_paramThread->requestRefresh();
        ui->statusLabel->setText(tr("正在刷新参数..."));
        ui->refreshButton->setEnabled(false);
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onRefreshButtonClicked 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), tr("刷新参数时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onRefreshButtonClicked 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("刷新参数时发生未知错误"));
    }
}

void MainWindow::onApplyButtonClicked()
{
    try {
        // 检查指针有效性
        if (!m_paramThread) {
            qCritical() << "[MainWindow] 错误：Nav2ParameterThread 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("参数线程未初始化"));
            return;
        }

        if (!m_paramThread->hasPendingChanges()) {
            QMessageBox::information(this, tr("提示"), tr("没有待应用的更改"));
            return;
        }

        auto reply = QMessageBox::question(this, tr("确认应用"),
            tr("确定要应用修改的参数到 ROS2 吗？"),
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            m_paramThread->requestApply();
            ui->statusLabel->setText(tr("正在应用参数..."));
            ui->applyButton->setEnabled(false);
        }
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onApplyButtonClicked 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), tr("应用参数时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onApplyButtonClicked 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("应用参数时发生未知错误"));
    }
}

void MainWindow::onResetButtonClicked()
{
    try {
        // 检查指针有效性
        if (!m_paramThread) {
            qCritical() << "[MainWindow] 错误：Nav2ParameterThread 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("参数线程未初始化"));
            return;
        }

        auto reply = QMessageBox::question(this, tr("确认重置"),
            tr("确定要将所有参数重置为默认值吗？\n重置后需要点击\"应用更改\"按钮。"),
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            m_paramThread->requestReset();
        }
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onResetButtonClicked 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), tr("重置参数时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onResetButtonClicked 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("重置参数时发生未知错误"));
    }
}

void MainWindow::onDiscardButtonClicked()
{
    try {
        // 检查指针有效性
        if (!m_paramThread) {
            qCritical() << "[MainWindow] 错误：Nav2ParameterThread 未初始化";
            QMessageBox::critical(this, tr("错误"), tr("参数线程未初始化"));
            return;
        }

        auto reply = QMessageBox::question(this, tr("确认放弃"),
            tr("确定要放弃所有未应用的修改吗？"),
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            m_paramThread->requestDiscard();
        }
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onDiscardButtonClicked 发生异常:" << e.what();
        QMessageBox::critical(this, tr("错误"), tr("放弃修改时发生错误:\n%1").arg(e.what()));
    } catch (...) {
        qCritical() << "[MainWindow] onDiscardButtonClicked 发生未知异常";
        QMessageBox::critical(this, tr("错误"), tr("放弃修改时发生未知错误"));
    }
}

void MainWindow::onParameterRefreshed(bool success, const QString& message)
{
    try {
        ui->statusLabel->setText(message);
        ui->refreshButton->setEnabled(true);

        if (success) {
            ui->statusLabel->setStyleSheet("color: green;");
            // 更新所有参数控件显示
            if (m_paramThread) {
                auto params = m_paramThread->getAllParams();
                for (auto it = params.begin(); it != params.end(); ++it) {
                    updateParameterValue(it.key(), it.value().currentValue);
                }
            }
        } else {
            ui->statusLabel->setStyleSheet("color: red;");
        }

        QTimer::singleShot(3000, this, [this]() {
            ui->statusLabel->setText(tr("就绪"));
            ui->statusLabel->setStyleSheet("");
        });
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onParameterRefreshed 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] onParameterRefreshed 发生未知异常";
    }
}

void MainWindow::onParameterApplied(bool success, const QString& message, const QStringList& appliedKeys)
{
    try {
        ui->statusLabel->setText(message);
        ui->applyButton->setEnabled(true);

        if (success) {
            ui->statusLabel->setStyleSheet("color: green;");
            // 更新已应用的参数控件显示
            if (m_paramThread) {
                for (const QString& key : appliedKeys) {
                    Nav2ParameterThread::ParamInfo info;
                    if (m_paramThread->getParamInfo(key, info)) {
                        updateParameterValue(key, info.currentValue);
                    }
                }
            }
        } else {
            ui->statusLabel->setStyleSheet("color: red;");
        }

        QTimer::singleShot(3000, this, [this]() {
            ui->statusLabel->setText(tr("就绪"));
            ui->statusLabel->setStyleSheet("");
        });
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onParameterApplied 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] onParameterApplied 发生未知异常";
    }
}

void MainWindow::onParameterOperationFinished(const QString& operation, bool success, const QString& message)
{
    try {
        ui->statusLabel->setText(message);

        if (success) {
            ui->statusLabel->setStyleSheet("color: green;");
            if (operation == "Reset" && m_paramThread) {
                // 重置后更新显示为 pendingValue
                auto params = m_paramThread->getAllParams();
                for (auto it = params.begin(); it != params.end(); ++it) {
                    updateParameterValue(it.key(), it.value().pendingValue);
                }
            }
        } else {
            ui->statusLabel->setStyleSheet("color: red;");
        }

        QTimer::singleShot(3000, this, [this]() {
            ui->statusLabel->setText(tr("就绪"));
            ui->statusLabel->setStyleSheet("");
        });
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onParameterOperationFinished 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] onParameterOperationFinished 发生未知异常";
    }
}

void MainWindow::onParameterValueChanged(double value)
{
    try {
        QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
        if (!spinBox) return;

        // 检查指针有效性
        if (!m_paramThread) {
            qWarning() << "[MainWindow] 警告：Nav2ParameterThread 未初始化";
            return;
        }

        QString key = spinBox->objectName();
        // 将控件名转换为参数 key
        key.replace("SpinBox", "");

        m_paramThread->setPendingValue(key, value);
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] onParameterValueChanged 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] onParameterValueChanged 发生未知异常";
    }
}

void MainWindow::updateParameterValue(const QString& key, const QVariant& value)
{
    try {
        QString objectName = key + "SpinBox";
        QDoubleSpinBox* spinBox = findChild<QDoubleSpinBox*>(objectName);
        if (spinBox) {
            spinBox->setValue(value.toDouble());
        }
    } catch (const std::exception& e) {
        qCritical() << "[MainWindow] updateParameterValue 发生异常:" << e.what();
    } catch (...) {
        qCritical() << "[MainWindow] updateParameterValue 发生未知异常";
    }
}

void MainWindow::addLogEntry(const LogEntry& entry)
{
    m_allLogs.append(entry);

    if (m_allLogs.size() > MAX_ALL_LOGS_SIZE) {
        int removeCount = m_allLogs.size() - MAX_ALL_LOGS_SIZE;
        for (int i = 0; i < removeCount; ++i) {
            m_allLogs.removeFirst();
        }
    }

    // 直接添加到模型，由LogFilterProxyModel负责过滤显示
    m_logTableModel->addLogEntry(entry);
    ui->logTableView->scrollToBottom();
}

QString MainWindow::getMapPathFromRosParam(rclcpp::Node::SharedPtr node)
{
    if (!node) return QString();
    node->declare_parameter("map_yaml_path", "");
    std::string map_path = node->get_parameter("map_yaml_path").as_string();
    if (map_path.empty()) {
        // 回退到默认路径
        return QStandardPaths::writableLocation(QStandardPaths::HomeLocation) +
               "/wheeltec_ros2/src/renwu/map/DisplaySimulationMap.yaml";
    }
    return QString::fromStdString(map_path);
}
