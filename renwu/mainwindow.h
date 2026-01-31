#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThreadPool>
#include <QStandardPaths>
#include <memory>
#include "robotstatusthread.h"
#include "navstatusthread.h"
#include "systemmonitorthread.h"
#include "logthread.h"
#include "logtablemodel.h"
#include "logfilterproxymodel.h"
#include "logquerytask.h"
#include "mapwidget.h"
#include "mapcache.h"
#include "nav2viewwidget.h"
#include "navigationactionclient.h"
#include "pathvisualizer.h"
#include "userstorageengine.h"
#include "userauthmanager.h"
#include "logindialog.h"
#include "usermanagementdialog.h"
#include "changepassworddialog.h"
#include "user.h"
#include "nav2parameterthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    /**
     * @brief 初始化主窗口
     * @return 初始化成功返回true，失败返回false
     * @details 两阶段初始化，避免构造函数中调用exit或close
     */
    bool initialize();


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

    // 日志过滤槽函数
    void onFilterChanged();

    // 地图相关槽函数
    void onMapClicked(double x, double y);
    void onLoadMapFromFile();

    void onStartNavigation();
    void onCancelNavigation();
    void onClearGoal();
    void onNavigationFeedback(double distanceRemaining, double navigationTime, int recoveries, double estimatedTimeRemaining);
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

    // Nav2ParameterThread 槽函数
    void onRefreshButtonClicked();
    void onApplyButtonClicked();
    void onResetButtonClicked();
    void onDiscardButtonClicked();
    void onParameterRefreshed(bool success, const QString& message);
    void onParameterApplied(bool success, const QString& message, const QStringList& appliedKeys);
    void onParameterOperationFinished(const QString& operation, bool success, const QString& message);
    void onParameterValueChanged(double value);

private:
    void initializeThreads();
    void connectSignals();
    void startAllThreads();
    void stopAllThreads();
    Q_INVOKABLE void refreshLogDisplay(bool autoScroll = true);
    Q_INVOKABLE bool shouldDisplayLog(int level) const;
    void updateParameterValue(const QString& key, const QVariant& value);
    void addLogEntry(const LogEntry& entry);

private:
    Ui::MainWindow *ui;
    
    static constexpr int THREAD_START_DELAY_MS = 100;     // 线程启动延迟时间
    static constexpr int THREAD_STOP_TIMEOUT_MS = 3000;   // 线程停止超时时间
    
    // 从ROS参数获取地图路径
    static QString getMapPathFromRosParam(rclcpp::Node::SharedPtr node);

    // 使用智能指针管理线程生命周期，避免内存泄漏和悬空指针
    std::unique_ptr<RobotStatusThread> m_robotStatusThread;
    std::unique_ptr<NavStatusThread> m_navStatusThread;
    std::unique_ptr<SystemMonitorThread> m_systemMonitorThread;
    std::unique_ptr<LogThread> m_logThread;
    std::unique_ptr<LogStorageEngine> m_logStorage;
    std::unique_ptr<LogTableModel> m_logTableModel;
    std::unique_ptr<LogFilterProxyModel> m_logFilterProxyModel;
    QList<LogEntry> m_allLogs;
    static constexpr int MAX_ALL_LOGS_SIZE = 10000;
    std::unique_ptr<Nav2ViewWidget> m_nav2ViewWidget;
    // std::unique_ptr<MapWidget> m_mapWidget;  // 保留备份
    std::unique_ptr<MapCache> m_mapCache;
    std::unique_ptr<NavigationActionClient> m_navigationClient;
    std::unique_ptr<PathVisualizer> m_pathVisualizer;
    double m_targetX;
    double m_targetY;
    double m_targetYaw;
    bool m_hasTarget;
    QDateTime m_startTime;
    double m_initialDistance;

    // 用户认证模块使用智能指针管理
    std::unique_ptr<UserStorageEngine> m_userStorageEngine;
    std::unique_ptr<UserAuthManager> m_userAuthManager;
    std::unique_ptr<LoginDialog> m_loginDialog;
    std::unique_ptr<UserManagementDialog> m_userManagementDialog;
    std::unique_ptr<Nav2ParameterThread> m_paramThread;
};

#endif // MAINWINDOW_H
