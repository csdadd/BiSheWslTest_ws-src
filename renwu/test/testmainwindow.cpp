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

void TestMainWindow::testQueryLogsAsync()
{
    QDateTime startTime = QDateTime::currentDateTime().addDays(-1);
    QDateTime endTime = QDateTime::currentDateTime();
    
    m_mainWindow->queryLogsAsync(startTime, endTime, -1, QString(), QString(), -1, 0);
    
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
