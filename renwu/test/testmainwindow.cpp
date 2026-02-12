#include "testmainwindow.h"

void TestMainWindow::initTestCase()
{
}

void TestMainWindow::cleanupTestCase()
{
}

void TestMainWindow::init()
{
    m_mainWindow = new MainWindow();
}

void TestMainWindow::cleanup()
{
    delete m_mainWindow;
    m_mainWindow = nullptr;
}

void TestMainWindow::testConstructor()
{
    QVERIFY(m_mainWindow != nullptr);
    QVERIFY(m_mainWindow->windowTitle().contains("MainWindow", Qt::CaseInsensitive));
}

void TestMainWindow::testUIComponents()
{
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("statusGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("batteryGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("positionGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("timeGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("mapGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("filterGroupBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QTableView*>("logTableView") != nullptr);

    // 验证日志级别复选框存在
    QVERIFY(m_mainWindow->findChild<QCheckBox*>("debugCheckBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QCheckBox*>("infoCheckBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QCheckBox*>("warningCheckBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QCheckBox*>("errorCheckBox") != nullptr);
    QVERIFY(m_mainWindow->findChild<QCheckBox*>("fatalCheckBox") != nullptr);
}

void TestMainWindow::testThreadInitialization()
{
    QVERIFY(m_mainWindow->findChild<QGroupBox*>("statusGroupBox") != nullptr);
}

void TestMainWindow::testSignalConnections()
{
    QPushButton* loadMapButton = m_mainWindow->findChild<QPushButton*>("loadMapButton");
    QVERIFY(loadMapButton != nullptr);

    QPushButton* btnStartNavigation = m_mainWindow->findChild<QPushButton*>("btnStartNavigation");
    QVERIFY(btnStartNavigation != nullptr);

    QPushButton* btnCancelNavigation = m_mainWindow->findChild<QPushButton*>("btnCancelNavigation");
    QVERIFY(btnCancelNavigation != nullptr);

    QPushButton* btnClearGoal = m_mainWindow->findChild<QPushButton*>("btnClearGoal");
    QVERIFY(btnClearGoal != nullptr);

    // 验证日志过滤复选框信号连接
    QCheckBox* debugCheckBox = m_mainWindow->findChild<QCheckBox*>("debugCheckBox");
    QVERIFY(debugCheckBox != nullptr);

    QCheckBox* infoCheckBox = m_mainWindow->findChild<QCheckBox*>("infoCheckBox");
    QVERIFY(infoCheckBox != nullptr);

    QCheckBox* warningCheckBox = m_mainWindow->findChild<QCheckBox*>("warningCheckBox");
    QVERIFY(warningCheckBox != nullptr);

    QCheckBox* errorCheckBox = m_mainWindow->findChild<QCheckBox*>("errorCheckBox");
    QVERIFY(errorCheckBox != nullptr);

    QCheckBox* fatalCheckBox = m_mainWindow->findChild<QCheckBox*>("fatalCheckBox");
    QVERIFY(fatalCheckBox != nullptr);
}

void TestMainWindow::testBatteryStatusSlot()
{
    QLabel* labelVoltage = m_mainWindow->findChild<QLabel*>("labelVoltage");
    QLabel* labelPercentage = m_mainWindow->findChild<QLabel*>("labelPercentage");
    QLabel* labelBatteryStatus = m_mainWindow->findChild<QLabel*>("labelBatteryStatus");
    
    QVERIFY(labelVoltage != nullptr);
    QVERIFY(labelPercentage != nullptr);
    QVERIFY(labelBatteryStatus != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onBatteryStatusReceived", 
                              Qt::DirectConnection,
                              Q_ARG(float, 12.5f),
                              Q_ARG(float, 85.0f));
    
    QTest::qWait(100);
}

void TestMainWindow::testPositionSlot()
{
    QLabel* labelX = m_mainWindow->findChild<QLabel*>("labelX");
    QLabel* labelY = m_mainWindow->findChild<QLabel*>("labelY");
    QLabel* labelYaw = m_mainWindow->findChild<QLabel*>("labelYaw");
    
    QVERIFY(labelX != nullptr);
    QVERIFY(labelY != nullptr);
    QVERIFY(labelYaw != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onPositionReceived",
                              Qt::DirectConnection,
                              Q_ARG(double, 1.5),
                              Q_ARG(double, 2.5),
                              Q_ARG(double, 45.0));
    
    QTest::qWait(100);
}

void TestMainWindow::testOdometrySlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onOdometryReceived",
                              Qt::DirectConnection,
                              Q_ARG(double, 1.5),
                              Q_ARG(double, 2.5),
                              Q_ARG(double, 45.0),
                              Q_ARG(double, 0.1),
                              Q_ARG(double, 0.0),
                              Q_ARG(double, 0.05));
    
    QTest::qWait(100);
}

void TestMainWindow::testSystemTimeSlot()
{
    QLabel* labelCurrentTime = m_mainWindow->findChild<QLabel*>("labelCurrentTime");
    
    QVERIFY(labelCurrentTime != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onSystemTimeReceived",
                              Qt::DirectConnection,
                              Q_ARG(QString, "2026-01-12 12:00:00"));
    
    QTest::qWait(100);
}

void TestMainWindow::testDiagnosticsSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onDiagnosticsReceived",
                              Qt::DirectConnection,
                              Q_ARG(QString, "OK"),
                              Q_ARG(int, 0),
                              Q_ARG(QString, "System normal"));
    
    QTest::qWait(100);
}

void TestMainWindow::testNavigationStatusSlot()
{
    QLabel* labelNavigationStatus = m_mainWindow->findChild<QLabel*>("labelNavigationStatus");
    
    QVERIFY(labelNavigationStatus != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onNavigationStatusReceived",
                              Qt::DirectConnection,
                              Q_ARG(int, 1),
                              Q_ARG(QString, "Navigating"));
    
    QTest::qWait(100);
}

void TestMainWindow::testNavigationFeedbackSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onNavigationFeedbackReceived",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Moving to target"));
    
    QTest::qWait(100);
}

void TestMainWindow::testNavigationPathSlot()
{
    QVector<QPointF> path;
    path.append(QPointF(0.0, 0.0));
    path.append(QPointF(1.0, 1.0));
    path.append(QPointF(2.0, 2.0));
    
    QMetaObject::invokeMethod(m_mainWindow, "onNavigationPathReceived",
                              Qt::DirectConnection,
                              Q_ARG(QVector<QPointF>, path));
    
    QTest::qWait(100);
}

void TestMainWindow::testLogMessageSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Test log message"),
                              Q_ARG(int, 1),
                              Q_ARG(QDateTime, QDateTime::currentDateTime()));
    
    QTest::qWait(100);
}

void TestMainWindow::testCollisionDetectedSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onCollisionDetected",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Collision detected at (1.0, 1.0)"));
    
    QTest::qWait(100);
}

void TestMainWindow::testAnomalyDetectedSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onAnomalyDetected",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Anomaly detected: battery low"));
    
    QTest::qWait(100);
}

void TestMainWindow::testBehaviorTreeLogSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onBehaviorTreeLogReceived",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Behavior tree: executing navigation"));
    
    QTest::qWait(100);
}

void TestMainWindow::testMapReceivedSlot()
{
    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    
    QMetaObject::invokeMethod(m_mainWindow, "onMapReceived",
                              Qt::DirectConnection,
                              Q_ARG(QImage, testImage),
                              Q_ARG(double, 0.05),
                              Q_ARG(double, -5.0),
                              Q_ARG(double, -5.0));
    
    QTest::qWait(100);
}

void TestMainWindow::testMapClickedSlot()
{
    QMetaObject::invokeMethod(m_mainWindow, "onMapClicked",
                              Qt::DirectConnection,
                              Q_ARG(double, 1.0),
                              Q_ARG(double, 2.0));
    
    QTest::qWait(100);
}

void TestMainWindow::testNavigationControlSlots()
{
    QPushButton* btnStartNavigation = m_mainWindow->findChild<QPushButton*>("btnStartNavigation");
    QPushButton* btnCancelNavigation = m_mainWindow->findChild<QPushButton*>("btnCancelNavigation");
    QPushButton* btnClearGoal = m_mainWindow->findChild<QPushButton*>("btnClearGoal");
    
    QVERIFY(btnStartNavigation != nullptr);
    QVERIFY(btnCancelNavigation != nullptr);
    QVERIFY(btnClearGoal != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onStartNavigation", Qt::DirectConnection);
    QTest::qWait(100);
    
    QMetaObject::invokeMethod(m_mainWindow, "onCancelNavigation", Qt::DirectConnection);
    QTest::qWait(100);
    
    QMetaObject::invokeMethod(m_mainWindow, "onClearGoal", Qt::DirectConnection);
    QTest::qWait(100);
}

void TestMainWindow::testUserPermissionSlots()
{
    QAction* actionUserManagement = m_mainWindow->findChild<QAction*>("actionUserManagement");
    QAction* actionChangePassword = m_mainWindow->findChild<QAction*>("actionChangePassword");
    QAction* actionLogout = m_mainWindow->findChild<QAction*>("actionLogout");
    
    QVERIFY(actionUserManagement != nullptr);
    QVERIFY(actionChangePassword != nullptr);
    QVERIFY(actionLogout != nullptr);
    
    QMetaObject::invokeMethod(m_mainWindow, "onUserManagement", Qt::DirectConnection);
    QTest::qWait(100);
    
    QMetaObject::invokeMethod(m_mainWindow, "onChangePassword", Qt::DirectConnection);
    QTest::qWait(100);
    
    QMetaObject::invokeMethod(m_mainWindow, "onLogout", Qt::DirectConnection);
    QTest::qWait(100);
}

void TestMainWindow::testUIPermissionControl()
{
    QMetaObject::invokeMethod(m_mainWindow, "updateUIBasedOnPermission", Qt::DirectConnection);
    QTest::qWait(100);
}

void TestMainWindow::testThreadLifecycle()
{
    QMetaObject::invokeMethod(m_mainWindow, "onConnectionStateChanged",
                              Qt::DirectConnection,
                              Q_ARG(bool, true));
    QTest::qWait(100);

    QMetaObject::invokeMethod(m_mainWindow, "onThreadStarted",
                              Qt::DirectConnection,
                              Q_ARG(QString, "TestThread"));
    QTest::qWait(100);

    QMetaObject::invokeMethod(m_mainWindow, "onThreadStopped",
                              Qt::DirectConnection,
                              Q_ARG(QString, "TestThread"));
    QTest::qWait(100);

    QMetaObject::invokeMethod(m_mainWindow, "onThreadError",
                              Qt::DirectConnection,
                              Q_ARG(QString, "Test error"));
    QTest::qWait(100);
}

void TestMainWindow::testLogFilterCheckBoxesUI()
{
    // 验证所有日志级别复选框存在
    QCheckBox* debugCheckBox = m_mainWindow->findChild<QCheckBox*>("debugCheckBox");
    QVERIFY(debugCheckBox != nullptr);

    QCheckBox* infoCheckBox = m_mainWindow->findChild<QCheckBox*>("infoCheckBox");
    QVERIFY(infoCheckBox != nullptr);

    QCheckBox* warningCheckBox = m_mainWindow->findChild<QCheckBox*>("warningCheckBox");
    QVERIFY(warningCheckBox != nullptr);

    QCheckBox* errorCheckBox = m_mainWindow->findChild<QCheckBox*>("errorCheckBox");
    QVERIFY(errorCheckBox != nullptr);

    QCheckBox* fatalCheckBox = m_mainWindow->findChild<QCheckBox*>("fatalCheckBox");
    QVERIFY(fatalCheckBox != nullptr);

    // 验证复选框默认状态为勾选
    QVERIFY(debugCheckBox->isChecked());
    QVERIFY(infoCheckBox->isChecked());
    QVERIFY(warningCheckBox->isChecked());
    QVERIFY(errorCheckBox->isChecked());
    QVERIFY(fatalCheckBox->isChecked());
}

void TestMainWindow::testLogFiltering()
{
    QCheckBox* debugCheckBox = m_mainWindow->findChild<QCheckBox*>("debugCheckBox");
    QCheckBox* infoCheckBox = m_mainWindow->findChild<QCheckBox*>("infoCheckBox");
    QCheckBox* warningCheckBox = m_mainWindow->findChild<QCheckBox*>("warningCheckBox");
    QCheckBox* errorCheckBox = m_mainWindow->findChild<QCheckBox*>("errorCheckBox");
    QCheckBox* fatalCheckBox = m_mainWindow->findChild<QCheckBox*>("fatalCheckBox");
    QTableView* logTableView = m_mainWindow->findChild<QTableView*>("logTableView");

    QVERIFY(debugCheckBox != nullptr);
    QVERIFY(infoCheckBox != nullptr);
    QVERIFY(warningCheckBox != nullptr);
    QVERIFY(errorCheckBox != nullptr);
    QVERIFY(fatalCheckBox != nullptr);
    QVERIFY(logTableView != nullptr);

    // 获取 LogFilterProxyModel
    auto* proxyModel = qobject_cast<QSortFilterProxyModel*>(logTableView->model());
    QVERIFY(proxyModel != nullptr);

    // 添加测试日志数据
    QDateTime now = QDateTime::currentDateTime();
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Debug message"),
                              Q_ARG(int, 0),  // LOG_DEBUG
                              Q_ARG(QDateTime, now));
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Info message"),
                              Q_ARG(int, 1),  // LOG_INFO
                              Q_ARG(QDateTime, now.addSecs(1)));
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Warning message"),
                              Q_ARG(int, 2),  // LOG_WARNING
                              Q_ARG(QDateTime, now.addSecs(2)));

    // 测试默认状态（所有复选框都勾选）- 应该显示所有日志
    QCOMPARE(proxyModel->rowCount(), 3);

    // 取消勾选 DEBUG 复选框 - 应该过滤掉 DEBUG 日志
    debugCheckBox->setChecked(false);
    QMetaObject::invokeMethod(m_mainWindow, "refreshLogDisplay", Qt::DirectConnection);
    QCOMPARE(proxyModel->rowCount(), 2);

    // 取消勾选 INFO 复选框 - 应该再过滤掉 INFO 日志
    infoCheckBox->setChecked(false);
    QMetaObject::invokeMethod(m_mainWindow, "refreshLogDisplay", Qt::DirectConnection);
    QCOMPARE(proxyModel->rowCount(), 1);

    // 恢复 DEBUG 复选框
    debugCheckBox->setChecked(true);
    QMetaObject::invokeMethod(m_mainWindow, "refreshLogDisplay", Qt::DirectConnection);
    QCOMPARE(proxyModel->rowCount(), 2);

    // 取消勾选 WARNING - 应该过滤掉所有日志
    warningCheckBox->setChecked(false);
    QMetaObject::invokeMethod(m_mainWindow, "refreshLogDisplay", Qt::DirectConnection);
    QCOMPARE(proxyModel->rowCount(), 1);

    // 恢复所有复选框
    debugCheckBox->setChecked(true);
    infoCheckBox->setChecked(true);
    warningCheckBox->setChecked(true);
    QMetaObject::invokeMethod(m_mainWindow, "refreshLogDisplay", Qt::DirectConnection);
    QCOMPARE(proxyModel->rowCount(), 3);
}

void TestMainWindow::testRefreshLogDisplay()
{
    // 模拟添加多条不同级别的日志
    QDateTime now = QDateTime::currentDateTime();

    // 添加5条不同级别的日志
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Debug message"),
                              Q_ARG(int, 0),  // LOG_DEBUG
                              Q_ARG(QDateTime, now));

    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Info message"),
                              Q_ARG(int, 1),  // LOG_INFO
                              Q_ARG(QDateTime, now.addSecs(1)));

    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Warning message"),
                              Q_ARG(int, 2),  // LOG_WARNING
                              Q_ARG(QDateTime, now.addSecs(2)));

    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Error message"),
                              Q_ARG(int, 3),  // LOG_ERROR
                              Q_ARG(QDateTime, now.addSecs(3)));

    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Fatal message"),
                              Q_ARG(int, 4),  // LOG_FATAL
                              Q_ARG(QDateTime, now.addSecs(4)));

    QTest::qWait(100);

    QTableView* logTableView = m_mainWindow->findChild<QTableView*>("logTableView");
    QVERIFY(logTableView != nullptr);

    int initialRowCount = logTableView->model()->rowCount();
    QVERIFY(initialRowCount >= 5);

    // 只保留 ERROR 和 FATAL 级别
    QCheckBox* debugCheckBox = m_mainWindow->findChild<QCheckBox*>("debugCheckBox");
    QCheckBox* infoCheckBox = m_mainWindow->findChild<QCheckBox*>("infoCheckBox");
    QCheckBox* warningCheckBox = m_mainWindow->findChild<QCheckBox*>("warningCheckBox");

    debugCheckBox->setChecked(false);
    infoCheckBox->setChecked(false);
    warningCheckBox->setChecked(false);

    QTest::qWait(100);

    // 验证只显示 ERROR 和 FATAL 级别日志
    int filteredRowCount = logTableView->model()->rowCount();
    QVERIFY(filteredRowCount < initialRowCount);

    // 恢复所有复选框
    debugCheckBox->setChecked(true);
    infoCheckBox->setChecked(true);
    warningCheckBox->setChecked(true);

    QTest::qWait(100);

    // 验证所有日志重新显示
    int restoredRowCount = logTableView->model()->rowCount();
    QVERIFY(restoredRowCount == initialRowCount);
}

void TestMainWindow::testOnFilterChanged()
{
    // 先添加一些测试日志
    QDateTime now = QDateTime::currentDateTime();
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Test message 1"),
                              Q_ARG(int, 0),  // LOG_DEBUG
                              Q_ARG(QDateTime, now));

    QTest::qWait(50);

    QTableView* logTableView = m_mainWindow->findChild<QTableView*>("logTableView");
    QVERIFY(logTableView != nullptr);

    int countBefore = logTableView->model()->rowCount();

    // 切换 DEBUG 复选框状态
    QCheckBox* debugCheckBox = m_mainWindow->findChild<QCheckBox*>("debugCheckBox");
    QVERIFY(debugCheckBox != nullptr);

    debugCheckBox->setChecked(false);
    QTest::qWait(50);

    int countAfter = logTableView->model()->rowCount();
    QVERIFY(countAfter < countBefore);

    // 恢复
    debugCheckBox->setChecked(true);
    QTest::qWait(50);

    int countRestored = logTableView->model()->rowCount();
    QVERIFY(countRestored == countBefore);
}

void TestMainWindow::testLogRealTimeFiltering()
{
    QTableView* logTableView = m_mainWindow->findChild<QTableView*>("logTableView");
    QVERIFY(logTableView != nullptr);

    QDateTime now = QDateTime::currentDateTime();

    // 取消 INFO 和 WARNING
    QCheckBox* infoCheckBox = m_mainWindow->findChild<QCheckBox*>("infoCheckBox");
    QCheckBox* warningCheckBox = m_mainWindow->findChild<QCheckBox*>("warningCheckBox");
    infoCheckBox->setChecked(false);
    warningCheckBox->setChecked(false);

    QTest::qWait(50);

    int countBefore = logTableView->model()->rowCount();

    // 添加一条 INFO 日志（不应该显示）
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Info message"),
                              Q_ARG(int, 1),  // LOG_INFO
                              Q_ARG(QDateTime, now));

    QTest::qWait(50);

    int countAfterInfo = logTableView->model()->rowCount();
    QVERIFY(countAfterInfo == countBefore);  // INFO 日志不应该显示

    // 添加一条 ERROR 日志（应该显示）
    QMetaObject::invokeMethod(m_mainWindow, "onLogMessageReceived", Qt::DirectConnection,
                              Q_ARG(QString, "Error message"),
                              Q_ARG(int, 3),  // LOG_ERROR
                              Q_ARG(QDateTime, now.addSecs(1)));

    QTest::qWait(50);

    int countAfterError = logTableView->model()->rowCount();
    QVERIFY(countAfterError > countBefore);  // ERROR 日志应该显示

    // 重新勾选 INFO，验证历史 INFO 日志恢复
    infoCheckBox->setChecked(true);
    QTest::qWait(50);

    int countRestored = logTableView->model()->rowCount();
    QVERIFY(countRestored > countAfterError);  // INFO 历史日志应该恢复

    // 恢复所有复选框
    warningCheckBox->setChecked(true);
}
