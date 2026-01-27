#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QFileDialog>
#include <QTimer>
#include <algorithm>
#include "maploader.h"
#include "mapconverter.h"
#include "mapmarker.h"
#include "roscontextmanager.h"
#include <nav_msgs/msg/path.hpp>

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
    , m_nav2ViewWidget(nullptr)
    // , m_mapWidget(nullptr)  // 保留备份
    , m_mapCache(nullptr)
    , m_navigationClient(nullptr)
    , m_pathVisualizer(nullptr)
    , m_targetX(0.0)
    , m_targetY(0.0)
    , m_targetYaw(0.0)
    , m_hasTarget(false)
    , m_startTime(QDateTime::currentDateTime())
    , m_userStorageEngine(nullptr)
    , m_userAuthManager(nullptr)
    , m_loginDialog(nullptr)
    , m_userManagementDialog(nullptr)
    , m_paramThread(nullptr)
{
    ui->setupUi(this);

    qDebug() << "[MainWindow] 正在初始化用户存储引擎...";
    m_userStorageEngine = new UserStorageEngine(this);
    if (!m_userStorageEngine->initialize()) {
        qCritical() << "[MainWindow] 错误：用户存储引擎初始化失败！";
        QMessageBox::critical(this, "错误", "用户存储引擎初始化失败！\n\n程序无法继续运行。");
        exit(1);
    }

    qDebug() << "[MainWindow] 用户存储引擎初始化成功";

    qDebug() << "[MainWindow] 正在初始化用户认证管理器...";
    m_userAuthManager = new UserAuthManager(this);
    if (!m_userAuthManager->initialize(m_userStorageEngine)) {
        qCritical() << "[MainWindow] 错误：用户认证管理器初始化失败！";
        QMessageBox::critical(this, "错误", "用户认证管理器初始化失败！\n\n程序无法继续运行。");
        exit(1);
    }

    qDebug() << "[MainWindow] 用户认证管理器初始化成功";

    qDebug() << "[MainWindow] 正在创建登录对话框...";
    m_loginDialog = new LoginDialog(m_userAuthManager, this);
    if (!m_loginDialog) {
        qCritical() << "[MainWindow] 错误：登录对话框创建失败！";
        QMessageBox::critical(this, "错误", "登录对话框创建失败！\n\n程序无法继续运行。");
        exit(1);
    }

    qDebug() << "[MainWindow] 登录对话框创建成功，正在显示登录界面...";
    int dialogResult = m_loginDialog->exec();

    if (dialogResult != QDialog::Accepted) {
        qWarning() << "[MainWindow] 警告：登录对话框被用户取消或关闭";
        if (!m_userAuthManager->isLoggedIn()) {
            qCritical() << "[MainWindow] 错误：用户未登录，程序将退出";
            QMessageBox::critical(this, "错误", "登录失败，程序将退出");
            exit(1);
        }
    }

    if (!m_userAuthManager->isLoggedIn()) {
        qCritical() << "[MainWindow] 错误：登录验证失败，程序将退出";
        QMessageBox::critical(this, "错误", "登录失败，程序将退出");
        exit(1);
    }

    qDebug() << "[MainWindow] 用户登录成功:" << m_userAuthManager->getCurrentUsername();

    m_logTableModel = new LogTableModel(this);
    m_logFilterProxyModel = new LogFilterProxyModel(this);
    m_logFilterProxyModel->setSourceModel(m_logTableModel);
    ui->logTableView->setModel(m_logFilterProxyModel);

    ui->logTableView->setColumnWidth(0, 180);
    ui->logTableView->setColumnWidth(1, 80);
    ui->logTableView->setColumnWidth(2, 120);
    ui->logTableView->horizontalHeader()->setStretchLastSection(true);

    // 初始化 ROS2 上下文
    ROSContextManager::instance().initialize();

    // 创建 Nav2ViewWidget
    std::string map_path = "/home/w/wheeltec_ros2/src/renwu/map/WHEELTEC.yaml";
    auto node = std::make_shared<rclcpp::Node>("nav2_view_node");
    m_nav2ViewWidget = new Nav2ViewWidget(map_path, node, this);
    ui->nav2MapLayout->addWidget(m_nav2ViewWidget);

    // m_mapWidget = new MapWidget(this);
    // ui->mapLayout->addWidget(m_mapWidget);  // 保留备份

    m_mapCache = new MapCache(10, this);

    m_navigationClient = new NavigationActionClient(this);
    // m_pathVisualizer = new PathVisualizer(m_mapWidget->scene(), this);  // 保留备份
    m_pathVisualizer = nullptr;  // Nav2ViewWidget 内部处理路径显示

    QTimer* currentTimeTimer = new QTimer(this);
    connect(currentTimeTimer, &QTimer::timeout, this, &MainWindow::updateCurrentTime);
    currentTimeTimer->start(1000);

    initializeThreads();
    connectSignals();
    startAllThreads();

    updateUIBasedOnPermission();
}

MainWindow::~MainWindow()
{
    stopAllThreads();

    delete m_robotStatusThread;
    delete m_navStatusThread;
    delete m_systemMonitorThread;
    delete m_logThread;
    delete m_mapThread;
    delete m_paramThread;
    delete m_nav2ViewWidget;
    // delete m_mapWidget;  // 保留备份
    delete m_mapCache;
    delete m_navigationClient;
    // delete m_pathVisualizer;  // 现在为 nullptr
    delete ui;
}

void MainWindow::initializeThreads()
{
    m_robotStatusThread = new RobotStatusThread(this);
    m_navStatusThread = new NavStatusThread(this);
    m_systemMonitorThread = new SystemMonitorThread(this);
    m_logThread = new LogThread(this);
    m_mapThread = new MapThread(this);
    m_paramThread = new Nav2ParameterThread(this);

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

    // RobotStatusThread -> MapWidget (保留备份)
    // connect(m_robotStatusThread, &RobotStatusThread::positionReceived,
    //         this, [this](double x, double y, double yaw) {
    //             if (m_mapWidget) {
    //                 m_mapWidget->updateRobotPose(x, y, yaw);
    //             }
    //         });

    // MapWidget -> MainWindow (保留备份)
    // connect(m_mapWidget, &MapWidget::mapClicked,
    //         this, &MainWindow::onMapClicked);

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

    // 连接按钮信号
    connect(ui->btnStartNavigation, &QPushButton::clicked, this, &MainWindow::onStartNavigation);
    connect(ui->btnCancelNavigation, &QPushButton::clicked, this, &MainWindow::onCancelNavigation);
    connect(ui->btnClearGoal, &QPushButton::clicked, this, &MainWindow::onClearGoal);

    // 用户菜单信号连接
    connect(ui->actionUserManagement, &QAction::triggered, this, &MainWindow::onUserManagement);
    connect(ui->actionChangePassword, &QAction::triggered, this, &MainWindow::onChangePassword);
    connect(ui->actionLogout, &QAction::triggered, this, &MainWindow::onLogout);

    // NavigationActionClient信号连接
    connect(m_navigationClient, &NavigationActionClient::goalAccepted,
            this, &MainWindow::onGoalAccepted);
    connect(m_navigationClient, &NavigationActionClient::goalRejected,
            this, &MainWindow::onGoalRejected);
    connect(m_navigationClient, &NavigationActionClient::feedbackReceived,
            this, &MainWindow::onNavigationFeedback);
    connect(m_navigationClient, &NavigationActionClient::resultReceived,
            this, &MainWindow::onNavigationResult);
    connect(m_navigationClient, &NavigationActionClient::goalCanceled,
            this, &MainWindow::onGoalCanceled);

    // Nav2ViewWidget信号连接
    connect(m_nav2ViewWidget, &Nav2ViewWidget::goalPosePreview,
            this, [this](double x, double y, double yaw) {
                m_targetX = x;
                m_targetY = y;
                m_targetYaw = yaw;
                m_hasTarget = true;
                ui->labelTargetPosition->setText(QString("(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
            });

    // Nav2ParameterThread 信号连接
    connect(m_paramThread, &Nav2ParameterThread::threadStarted,
            this, &MainWindow::onThreadStarted);
    connect(m_paramThread, &Nav2ParameterThread::threadStopped,
            this, &MainWindow::onThreadStopped);
    connect(m_paramThread, &Nav2ParameterThread::threadError,
            this, &MainWindow::onThreadError);
    connect(m_paramThread, &Nav2ParameterThread::parameterRefreshed,
            this, &MainWindow::onParameterRefreshed);
    connect(m_paramThread, &Nav2ParameterThread::parameterApplied,
            this, &MainWindow::onParameterApplied);
    connect(m_paramThread, &Nav2ParameterThread::operationFinished,
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
    // 启动顺序：日志 -> 系统监控 -> 机器人状态 -> 导航状态 -> 地图 -> 参数
    m_logThread->start();
    QThread::msleep(100);

    m_systemMonitorThread->start();
    QThread::msleep(100);

    m_robotStatusThread->start();
    QThread::msleep(100);

    m_navStatusThread->start();
    QThread::msleep(100);

    m_mapThread->start();
    QThread::msleep(100);

    m_paramThread->start();
}

void MainWindow::stopAllThreads()
{
    // 停止顺序与启动相反
    if (m_paramThread && m_paramThread->isRunning()) {
        m_paramThread->stopThread();
        m_paramThread->wait(3000);
    }

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
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Battery");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onPositionReceived(double x, double y, double yaw)
{
    ui->labelX->setText(QString("%1 m").arg(x, 0, 'f', 3));
    ui->labelY->setText(QString("%1 m").arg(y, 0, 'f', 3));
    
    double yaw_degrees = yaw * 180.0 / M_PI;
    ui->labelYaw->setText(QString("%1 °").arg(yaw_degrees, 0, 'f', 1));

    QString message = QString("位置信息 - X: %1, Y: %2, Yaw: %3").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Position");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega)
{
    if (ui->labelX->text() == "-- m") {
        ui->labelX->setText(QString("%1 m").arg(x, 0, 'f', 3));
        ui->labelY->setText(QString("%1 m").arg(y, 0, 'f', 3));

        double yaw_degrees = yaw * 180.0 / M_PI;
        ui->labelYaw->setText(QString("%1 °").arg(yaw_degrees, 0, 'f', 1));
    }

    QString message = QString("里程计 - 位置(X:%1, Y:%2, Yaw:%3), 速度(vx:%4, vy:%5, omega:%6)")
        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2)
        .arg(vx, 0, 'f', 2).arg(vy, 0, 'f', 2).arg(omega, 0, 'f', 2);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "RobotStatus", "Odometry");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onSystemTimeReceived(const QString& time)
{
    ui->labelCurrentTime->setText(time);
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
    m_allLogs.append(entry);
    if (shouldDisplayLog(logLevel)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

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
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

    ui->statusbar->showMessage(QString("导航: %1 - %2").arg(statusStr).arg(message), 3000);
}

void MainWindow::onNavigationPathReceived(const QVector<QPointF>& path)
{
    QString message = QString("导航路径 - 路径点数: %1").arg(path.size());
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "NavStatus", "Navigation");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

    // Nav2ViewWidget 内部订阅 /plan 话题处理路径显示
    // if (m_pathVisualizer && m_mapWidget) {
    //     nav_msgs::msg::Path rosPath;
    //     rosPath.poses.reserve(path.size());
    //
    //     for (const QPointF& point : path) {
    //         geometry_msgs::msg::PoseStamped pose;
    //         pose.header.frame_id = "map";
    //         pose.pose.position.x = point.x();
    //         pose.pose.position.y = point.y();
    //         pose.pose.position.z = 0.0;
    //         rosPath.poses.push_back(pose);
    //     }
    //
    //     double resolution = 0.05;
    //     double originX = 0.0;
    //     double originY = 0.0;
    //     m_pathVisualizer->updatePath(rosPath, resolution, QPointF(originX, originY));
    // }
}

// ==================== SystemMonitorThread槽函数 ====================

void MainWindow::onLogMessageReceived(const QString& message, int level, const QDateTime& timestamp)
{
    LogEntry entry(message, level, timestamp, "SystemMonitor", "ROS");

    // 1. 添加到 allLogs 完整列表
    m_allLogs.append(entry);

    // 2. 根据当前勾选的级别，判断是否添加到显示列表
    if (shouldDisplayLog(level)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onCollisionDetected(const QString& message)
{
    QString logMessage = QString("碰撞检测 - %1").arg(message);
    LogEntry entry(logMessage, LOG_ERROR, QDateTime::currentDateTime(), "SystemMonitor", "Collision");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_ERROR)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

    ui->statusbar->showMessage(QString("警告: %1").arg(message), 5000);
}

void MainWindow::onAnomalyDetected(const QString& message)
{
    QString logMessage = QString("异常检测 - %1").arg(message);
    LogEntry entry(logMessage, LOG_WARNING, QDateTime::currentDateTime(), "SystemMonitor", "Anomaly");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_WARNING)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

    ui->statusbar->showMessage(QString("异常: %1").arg(message), 5000);
}

void MainWindow::onBehaviorTreeLogReceived(const QString& log)
{
    QString message = QString("行为树日志 - %1").arg(log);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "SystemMonitor", "BehaviorTree");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

// ==================== LogThread槽函数 ====================

void MainWindow::onLogFileChanged(const QString& filePath)
{
    QString message = QString("日志文件变更 - 新文件: %1").arg(filePath);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "LogThread", "File");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

// ==================== 线程状态槽函数 ====================

void MainWindow::onConnectionStateChanged(bool connected)
{
    QString status = connected ? "已连接" : "已断开";
    ui->statusbar->showMessage(QString("ROS连接状态: %1").arg(status), 3000);

    QString message = QString("连接状态变更 - %1").arg(status);
    int logLevel = connected ? LOG_INFO : LOG_WARNING;
    LogEntry entry(message, logLevel, QDateTime::currentDateTime(), "System", "Connection");
    m_allLogs.append(entry);
    if (shouldDisplayLog(logLevel)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onThreadStarted(const QString& threadName)
{
    QString message = QString("线程启动 - %1").arg(threadName);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "System", "Thread");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onThreadStopped(const QString& threadName)
{
    QString message = QString("线程停止 - %1").arg(threadName);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "System", "Thread");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
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
    if (ui->debugCheckBox->isChecked()) enabledLevels << LOG_DEBUG;
    if (ui->infoCheckBox->isChecked()) enabledLevels << LOG_INFO;
    if (ui->warningCheckBox->isChecked()) enabledLevels << LOG_WARNING;
    if (ui->errorCheckBox->isChecked()) enabledLevels << LOG_ERROR;
    if (ui->fatalCheckBox->isChecked()) enabledLevels << LOG_FATAL;

    // 清空当前显示
    m_logTableModel->clearLogs();

    // 从 allLogs 中筛选符合条件的日志
    QVector<LogEntry> filteredLogs;
    for (const auto& log : m_allLogs) {
        if (enabledLevels.contains(log.level)) {
            filteredLogs.append(log);
        }
    }

    if (!filteredLogs.isEmpty()) {
        m_logTableModel->addLogEntries(filteredLogs);
    }

    if (autoScroll) {
        ui->logTableView->scrollToBottom();
    }
}

bool MainWindow::shouldDisplayLog(int level) const
{
    switch (level) {
        case LOG_DEBUG:   return ui->debugCheckBox->isChecked();
        case LOG_INFO:    return ui->infoCheckBox->isChecked();
        case LOG_WARNING: return ui->warningCheckBox->isChecked();
        case LOG_ERROR:   return ui->errorCheckBox->isChecked();
        case LOG_FATAL:   return ui->fatalCheckBox->isChecked();
        default:          return false;
    }
}

// ==================== 地图相关槽函数 ====================

void MainWindow::onMapReceived(const QImage& mapImage, double resolution, double originX, double originY)
{
    // Nav2ViewWidget 从 YAML 文件加载地图，不需要处理 /map 话题
    // if (m_mapWidget) {
    //     m_mapWidget->setMapImage(mapImage, resolution, originX, originY);
    // }
    Q_UNUSED(mapImage)
    Q_UNUSED(resolution)
    Q_UNUSED(originX)
    Q_UNUSED(originY)
}

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
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Map", "Click");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onMapConnectionStateChanged(bool connected)
{
    QString status = connected ? "已连接" : "已断开";
    QString message = QString("地图连接状态: %1").arg(status);
    int logLevel = connected ? LOG_INFO : LOG_WARNING;
    LogEntry entry(message, logLevel, QDateTime::currentDateTime(), "Map", "Connection");
    m_allLogs.append(entry);
    if (shouldDisplayLog(logLevel)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

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

    // 删除旧的 Nav2ViewWidget
    delete m_nav2ViewWidget;

    // 创建新的 Nav2ViewWidget 加载新地图
    auto node = std::make_shared<rclcpp::Node>("nav2_view_node");
    m_nav2ViewWidget = new Nav2ViewWidget(filePath.toStdString(), node, this);
    ui->nav2MapLayout->addWidget(m_nav2ViewWidget);

    QString message = QString("地图文件加载成功 - 文件: %1").arg(filePath);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Map", "FileLoad");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }

    ui->statusbar->showMessage(QString("地图已加载: %1").arg(QFileInfo(filePath).fileName()), 3000);
}

void MainWindow::onStartNavigation()
{
    if (!m_hasTarget) {
        QMessageBox::warning(this, "提示", "请设置导航目标");
        return;
    }

    // 调用 Nav2ViewWidget 发布目标到 /goal_pose 话题
    m_nav2ViewWidget->publishCurrentGoal();

    ui->labelNavigationStatus->setText("导航中...");
    ui->labelNavigationStatus->setStyleSheet("color: blue;");
}

void MainWindow::onCancelNavigation()
{
    if (!m_navigationClient->isNavigating()) {
        QMessageBox::information(this, "提示", "无导航任务");
        return;
    }

    bool success = m_navigationClient->cancelGoal();
    if (!success) {
        QMessageBox::warning(this, "警告", "取消导航失败");
        return;
    }

    m_nav2ViewWidget->clearGoal();

    ui->labelNavigationStatus->setText("已取消");
    ui->labelNavigationStatus->setStyleSheet("color: orange;");
}

void MainWindow::onClearGoal()
{
    m_hasTarget = false;
    m_targetX = 0.0;
    m_targetY = 0.0;
    m_targetYaw = 0.0;

    // 清除 Nav2ViewWidget 的目标状态
    m_nav2ViewWidget->clearGoal();

    ui->labelTargetPosition->setText("未设置");
    ui->labelNavigationStatus->setText("空闲");
    ui->labelNavigationStatus->setStyleSheet("color: gray;");
    ui->labelDistanceRemaining->setText("-- m");
    ui->labelNavigationTime->setText("-- s");
    ui->labelRecoveries->setText("0");
}

void MainWindow::onNavigationFeedback(double distanceRemaining, double navigationTime, int recoveries)
{
    ui->labelDistanceRemaining->setText(QString::number(distanceRemaining, 'f', 2) + " m");
    ui->labelNavigationTime->setText(QString::number(navigationTime, 'f', 1) + " s");
    ui->labelRecoveries->setText(QString::number(recoveries));

    QString message = QString("导航反馈 - 剩余距离: %1m, 导航时间: %2s, 恢复次数: %3")
        .arg(distanceRemaining, 0, 'f', 2)
        .arg(navigationTime, 0, 'f', 1)
        .arg(recoveries);
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Navigation", "Feedback");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onNavigationResult(bool success, const QString& message)
{
    if (success) {
        QMessageBox::information(this, "导航完成", "导航成功到达目标点");
        ui->labelNavigationStatus->setText("导航成功");
        ui->labelNavigationStatus->setStyleSheet("color: green;");
    } else {
        QMessageBox::warning(this, "导航失败", message);
        ui->labelNavigationStatus->setText("导航失败");
        ui->labelNavigationStatus->setStyleSheet("color: red;");
    }

    QString logMessage = QString("导航结果 - %1: %2").arg(success ? "成功" : "失败").arg(message);
    int logLevel = success ? LOG_INFO : LOG_WARNING;
    LogEntry entry(logMessage, logLevel, QDateTime::currentDateTime(), "Navigation", "Result");
    m_allLogs.append(entry);
    if (shouldDisplayLog(logLevel)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onGoalAccepted()
{
    ui->labelNavigationStatus->setText("导航中...");
    ui->labelNavigationStatus->setStyleSheet("color: blue;");

    QString message = "导航目标已被接受，开始导航";
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Navigation", "Goal");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
    ui->statusbar->showMessage(message, 3000);
}

void MainWindow::onGoalRejected(const QString& reason)
{
    QMessageBox::warning(this, "目标被拒绝", reason);
    ui->labelNavigationStatus->setText("目标被拒绝");
    ui->labelNavigationStatus->setStyleSheet("color: orange;");

    QString message = QString("导航目标被拒绝 - 原因: %1").arg(reason);
    LogEntry entry(message, LOG_WARNING, QDateTime::currentDateTime(), "Navigation", "Goal");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_WARNING)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
}

void MainWindow::onGoalCanceled()
{
    QMessageBox::information(this, "导航已取消", "导航目标已取消");
    ui->labelNavigationStatus->setText("已取消");
    ui->labelNavigationStatus->setStyleSheet("color: gray;");

    QString message = "导航目标已取消";
    LogEntry entry(message, LOG_INFO, QDateTime::currentDateTime(), "Navigation", "Goal");
    m_allLogs.append(entry);
    if (shouldDisplayLog(LOG_INFO)) {
        m_logTableModel->addLogEntry(entry);
        ui->logTableView->scrollToBottom();
    }
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
    qDebug() << "[MainWindow] Login successful for user:" << user.username;
    updateUIBasedOnPermission();
}

void MainWindow::onLoginFailed(const QString& reason)
{
    qDebug() << "[MainWindow] Login failed:" << reason;
}

void MainWindow::onLogout()
{
    m_userAuthManager->logout();
    m_loginDialog->exec();

    if (!m_userAuthManager->isLoggedIn()) {
        QMessageBox::critical(this, "错误", "登录失败，程序将退出");
        exit(1);
    }

    updateUIBasedOnPermission();
}

void MainWindow::onUserManagement()
{
    if (!m_userAuthManager->canAdmin()) {
        QMessageBox::warning(this, "权限不足", "您没有权限访问用户管理功能");
        return;
    }

    m_userManagementDialog = new UserManagementDialog(m_userAuthManager, this);
    m_userManagementDialog->exec();
    delete m_userManagementDialog;
    m_userManagementDialog = nullptr;
}

void MainWindow::onChangePassword()
{
    ChangePasswordDialog dialog(ChangePasswordDialog::Mode::SelfChange, m_userAuthManager, "", this);
    dialog.exec();
}

void MainWindow::updateUIBasedOnPermission()
{
    UserPermission permission = m_userAuthManager->getCurrentPermission();

    bool canOperate = m_userAuthManager->canOperate();
    bool canAdmin = m_userAuthManager->canAdmin();

    ui->btnStartNavigation->setEnabled(canOperate);
    ui->btnCancelNavigation->setEnabled(canOperate);
    ui->btnClearGoal->setEnabled(canOperate);

    QString permissionText;
    switch (permission) {
        case UserPermission::VIEWER:
            permissionText = "查看者";
            break;
        case UserPermission::OPERATOR:
            permissionText = "操作员";
            break;
        case UserPermission::ADMIN:
            permissionText = "管理员";
            break;
    }

    QString statusText = QString("当前用户: %1 (%2)").arg(m_userAuthManager->getCurrentUsername()).arg(permissionText);
    ui->statusbar->showMessage(statusText, 0);

    qDebug() << "[MainWindow] UI updated based on permission:" << permissionText;
}

// ==================== Nav2ParameterThread 槽函数 ====================

void MainWindow::onRefreshButtonClicked()
{
    m_paramThread->requestRefresh();
    ui->statusLabel->setText("正在刷新参数...");
    ui->refreshButton->setEnabled(false);
}

void MainWindow::onApplyButtonClicked()
{
    if (!m_paramThread->hasPendingChanges()) {
        QMessageBox::information(this, "提示", "没有待应用的更改");
        return;
    }

    auto reply = QMessageBox::question(this, "确认应用",
        "确定要应用修改的参数到 ROS2 吗？",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        m_paramThread->requestApply();
        ui->statusLabel->setText("正在应用参数...");
        ui->applyButton->setEnabled(false);
    }
}

void MainWindow::onResetButtonClicked()
{
    auto reply = QMessageBox::question(this, "确认重置",
        "确定要将所有参数重置为默认值吗？\n重置后需要点击\"应用更改\"按钮。",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        m_paramThread->requestReset();
    }
}

void MainWindow::onDiscardButtonClicked()
{
    auto reply = QMessageBox::question(this, "确认放弃",
        "确定要放弃所有未应用的修改吗？",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        m_paramThread->requestDiscard();
    }
}

void MainWindow::onParameterRefreshed(bool success, const QString& message)
{
    ui->statusLabel->setText(message);
    ui->refreshButton->setEnabled(true);

    if (success) {
        ui->statusLabel->setStyleSheet("color: green;");
        // 更新所有参数控件显示
        auto params = m_paramThread->getAllParams();
        for (auto it = params.begin(); it != params.end(); ++it) {
            updateParameterValue(it.key(), it.value().currentValue);
        }
    } else {
        ui->statusLabel->setStyleSheet("color: red;");
    }

    QTimer::singleShot(3000, this, [this]() {
        ui->statusLabel->setText("就绪");
        ui->statusLabel->setStyleSheet("");
    });
}

void MainWindow::onParameterApplied(bool success, const QString& message, const QStringList& appliedKeys)
{
    ui->statusLabel->setText(message);
    ui->applyButton->setEnabled(true);

    if (success) {
        ui->statusLabel->setStyleSheet("color: green;");
        // 更新已应用的参数控件显示
        for (const QString& key : appliedKeys) {
            Nav2ParameterThread::ParamInfo info;
            if (m_paramThread->getParamInfo(key, info)) {
                updateParameterValue(key, info.currentValue);
            }
        }
    } else {
        ui->statusLabel->setStyleSheet("color: red;");
    }

    QTimer::singleShot(3000, this, [this]() {
        ui->statusLabel->setText("就绪");
        ui->statusLabel->setStyleSheet("");
    });
}

void MainWindow::onParameterOperationFinished(const QString& operation, bool success, const QString& message)
{
    ui->statusLabel->setText(message);

    if (success) {
        ui->statusLabel->setStyleSheet("color: green;");
        if (operation == "Reset") {
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
        ui->statusLabel->setText("就绪");
        ui->statusLabel->setStyleSheet("");
    });
}

void MainWindow::onParameterValueChanged(double value)
{
    QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spinBox) return;

    QString key = spinBox->objectName();
    // 将控件名转换为参数 key
    key.replace("SpinBox", "");

    m_paramThread->setPendingValue(key, value);
}

void MainWindow::updateParameterValue(const QString& key, const QVariant& value)
{
    QString objectName = key + "SpinBox";
    QDoubleSpinBox* spinBox = findChild<QDoubleSpinBox*>(objectName);
    if (spinBox) {
        spinBox->setValue(value.toDouble());
    }
}
