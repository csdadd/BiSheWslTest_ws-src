#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_robotStatusThread(nullptr)
    , m_navStatusThread(nullptr)
    , m_systemMonitorThread(nullptr)
    , m_logThread(nullptr)
{
    ui->setupUi(this);

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
    delete ui;
}

void MainWindow::initializeThreads()
{
    m_robotStatusThread = new RobotStatusThread(this);
    m_navStatusThread = new NavStatusThread(this);
    m_systemMonitorThread = new SystemMonitorThread(this);
    m_logThread = new LogThread(this);
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

    // RobotStatusThread的诊断信息转发给SystemMonitorThread
    connect(m_robotStatusThread, &RobotStatusThread::diagnosticsReceived,
            m_systemMonitorThread, &SystemMonitorThread::onDiagnosticsReceived);

    // LogThread信号连接
    connect(m_logThread, &LogThread::logFileChanged,
            this, &MainWindow::onLogFileChanged);

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
}

void MainWindow::startAllThreads()
{
    // 启动顺序：日志 -> 系统监控 -> 机器人状态 -> 导航状态
    m_logThread->start();
    QThread::msleep(100);

    m_systemMonitorThread->start();
    QThread::msleep(100);

    m_robotStatusThread->start();
    QThread::msleep(100);

    m_navStatusThread->start();
}

void MainWindow::stopAllThreads()
{
    // 停止顺序与启动相反
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
    qDebug() << "Battery:" << voltage << "V," << percentage << "%";
}

void MainWindow::onPositionReceived(double x, double y, double yaw)
{
    qDebug() << "Position:" << x << y << yaw;
}

void MainWindow::onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega)
{
    qDebug() << "Odometry:" << x << y << yaw << vx << vy << omega;
}

void MainWindow::onSystemTimeReceived(const QString& time)
{
    qDebug() << "System Time:" << time;
}

void MainWindow::onDiagnosticsReceived(const QString& status, int level, const QString& message)
{
    qDebug() << "Diagnostics:" << status << "level:" << level << message;
}

// ==================== NavStatusThread槽函数 ====================

void MainWindow::onNavigationStatusReceived(int status, const QString& message)
{
    qDebug() << "Navigation Status:" << status << message;
}

void MainWindow::onNavigationFeedbackReceived(const QString& feedback)
{
    qDebug() << "Navigation Feedback:" << feedback;
}

void MainWindow::onNavigationPathReceived(const QVector<QPointF>& path)
{
    qDebug() << "Navigation Path:" << path.size() << "points";
}

// ==================== SystemMonitorThread槽函数 ====================

void MainWindow::onLogMessageReceived(const QString& message, int level, const QDateTime& timestamp)
{
    qDebug() << "Log:" << timestamp.toString() << message;
}

void MainWindow::onCollisionDetected(const QString& message)
{
    qDebug() << "Collision:" << message;
}

void MainWindow::onAnomalyDetected(const QString& message)
{
    qDebug() << "Anomaly:" << message;
}

void MainWindow::onBehaviorTreeLogReceived(const QString& log)
{
    qDebug() << "Behavior Tree:" << log;
}

// ==================== LogThread槽函数 ====================

void MainWindow::onLogFileChanged(const QString& filePath)
{
    qDebug() << "Log File:" << filePath;
}

// ==================== 线程状态槽函数 ====================

void MainWindow::onConnectionStateChanged(bool connected)
{
    qDebug() << "Connection State:" << (connected ? "Connected" : "Disconnected");
}

void MainWindow::onThreadStarted(const QString& threadName)
{
    qDebug() << "Thread started:" << threadName;
}

void MainWindow::onThreadStopped(const QString& threadName)
{
    qDebug() << "Thread stopped:" << threadName;
}

void MainWindow::onThreadError(const QString& error)
{
    qDebug() << "Thread error:" << error;
}
