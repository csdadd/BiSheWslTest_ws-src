#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThreadPool>
#include "robotstatusthread.h"
#include "navstatusthread.h"
#include "systemmonitorthread.h"
#include "logthread.h"
#include "logtablemodel.h"
#include "logfilterproxymodel.h"
#include "logquerytask.h"
#include "mapthread.h"
#include "mapwidget.h"
#include "mapcache.h"
#include "navigationactionclient.h"
#include "pathvisualizer.h"
#include "userstorageengine.h"
#include "userauthmanager.h"
#include "logindialog.h"
#include "usermanagementdialog.h"
#include "user.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void queryLogsAsync(const QDateTime& startTime = QDateTime(),
                       const QDateTime& endTime = QDateTime(),
                       int minLevel = -1,
                       const QString& source = QString(),
                       const QString& keyword = QString(),
                       int limit = -1,
                       int offset = 0);

private slots:
    // RobotStatusThread槽函数
    void onBatteryStatusReceived(float voltage, float percentage);
    void onPositionReceived(double x, double y, double yaw);
    void onOdometryReceived(double x, double y, double yaw, double vx, double vy, double omega);
    void onSystemTimeReceived(const QString& time);
    void onDiagnosticsReceived(const QString& status, int level, const QString& message);
    void updateCurrentTime();

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

    // 日志查询槽函数
    void onQueryCompleted(const QVector<StorageLogEntry>& results);
    void onQueryFailed(const QString& error);

    // 日志过滤槽函数
    void onFilterChanged();
    void onQueryButtonClicked();
    void onClearFilterButtonClicked();
    void onRefreshButtonClicked();

    // 地图相关槽函数
    void onMapReceived(const QImage& mapImage, double resolution, double originX, double originY);
    void onMapClicked(double x, double y);
    void onMapConnectionStateChanged(bool connected);
    void onLoadMapFromFile();

    void onStartNavigation();
    void onCancelNavigation();
    void onClearGoal();
    void onNavigationFeedback(double distanceRemaining, double navigationTime, int recoveries);
    void onNavigationResult(bool success, const QString& message);
    void onGoalAccepted();
    void onGoalRejected(const QString& reason);
    void onGoalCanceled();

    // 用户权限管理槽函数
    void onLoginSuccess(const User& user);
    void onLoginFailed(const QString& reason);
    void onLogout();
    void onUserManagement();
    void onChangePassword();
    void updateUIBasedOnPermission();

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
    LogTableModel* m_logTableModel;
    LogFilterProxyModel* m_logFilterProxyModel;
    QThreadPool* m_threadPool;
    MapThread* m_mapThread;
    MapWidget* m_mapWidget;
    MapCache* m_mapCache;
    NavigationActionClient* m_navigationClient;
    PathVisualizer* m_pathVisualizer;
    double m_targetX;
    double m_targetY;
    double m_targetYaw;
    bool m_hasTarget;
    QDateTime m_startTime;

    UserStorageEngine* m_userStorageEngine;
    UserAuthManager* m_userAuthManager;
    LoginDialog* m_loginDialog;
    UserManagementDialog* m_userManagementDialog;
};

#endif // MAINWINDOW_H
