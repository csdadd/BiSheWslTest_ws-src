#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "robotstatusthread.h"
#include "navstatusthread.h"
#include "systemmonitorthread.h"
#include "logthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // RobotStatusThread槽函数
    void onBatteryStatusReceived(float voltage, float percentage);
    void onPositionReceived(double x, double y, double yaw);
    void onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega);
    void onSystemTimeReceived(const QString& time);
    void onDiagnosticsReceived(const QString& status, int level, const QString& message);

    // NavStatusThread槽函数
    void onNavigationStatusReceived(int status, const QString& message);
    void onNavigationFeedbackReceived(const QString& feedback);
    void onNavigationPathReceived(const QVector<QPointF>& path);

    // SystemMonitorThread槽函数
    void onLogMessageReceived(const QString& message, int level, const QDateTime& timestamp);
    void onCollisionDetected(const QString& message);
    void onAnomalyDetected(const QString& message);
    void onBehaviorTreeLogReceived(const QString& log);

    // LogThread槽函数
    void onLogFileChanged(const QString& filePath);

    // 线程状态槽函数
    void onConnectionStateChanged(bool connected);
    void onThreadStarted(const QString& threadName);
    void onThreadStopped(const QString& threadName);
    void onThreadError(const QString& error);

private:
    void initializeThreads();
    void connectSignals();
    void startAllThreads();
    void stopAllThreads();

private:
    Ui::MainWindow *ui;

    RobotStatusThread* m_robotStatusThread;
    NavStatusThread* m_navStatusThread;
    SystemMonitorThread* m_systemMonitorThread;
    LogThread* m_logThread;
};

#endif // MAINWINDOW_H
