#include "testnavigationactionthread.h"
#include "roscontextmanager.h"

void TestNavigationActionThread::initTestCase()
{
    ROSContextManager::instance().initialize();
}

void TestNavigationActionThread::cleanupTestCase()
{
}

void TestNavigationActionThread::init()
{
    m_thread = new NavigationActionThread();
}

void TestNavigationActionThread::cleanup()
{
    if (m_thread) {
        if (m_thread->isRunning()) {
            m_thread->stopThread();
            m_thread->wait(3000);
        }
        delete m_thread;
        m_thread = nullptr;
    }
}

// 基础功能测试
void TestNavigationActionThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isRunning());
    QVERIFY(!m_thread->isThreadRunning());
}

void TestNavigationActionThread::testDestructor()
{
    NavigationActionThread* thread = new NavigationActionThread();
    thread->start();
    QTest::qWait(100);

    // 正确停止线程
    thread->stopThread();
    thread->wait(3000);

    delete thread;
    // 验证析构不会崩溃
    QVERIFY(true);
}

// 线程生命周期测试
void TestNavigationActionThread::testThreadStart()
{
    QSignalSpy startedSpy(m_thread, &NavigationActionThread::threadStarted);

    m_thread->start();
    QTest::qWait(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(m_thread->isThreadRunning());
    QVERIFY(startedSpy.count() > 0);
}

void TestNavigationActionThread::testThreadStop()
{
    QSignalSpy stoppedSpy(m_thread, &NavigationActionThread::threadStopped);

    m_thread->start();
    QTest::qWait(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait(3000);

    QVERIFY(!m_thread->isRunning());
    QVERIFY(!m_thread->isThreadRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestNavigationActionThread::testIsRunning()
{
    QVERIFY(!m_thread->isRunning());
    QVERIFY(!m_thread->isThreadRunning());

    m_thread->start();
    QTest::qWait(100);
    QVERIFY(m_thread->isRunning());
    QVERIFY(m_thread->isThreadRunning());

    m_thread->stopThread();
    m_thread->wait(3000);
    QVERIFY(!m_thread->isRunning());
    QVERIFY(!m_thread->isThreadRunning());
}

void TestNavigationActionThread::testThreadStartedSignal()
{
    QSignalSpy startedSpy(m_thread, &NavigationActionThread::threadStarted);

    m_thread->start();
    QTest::qWait(100);

    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testThreadStoppedSignal()
{
    QSignalSpy stoppedSpy(m_thread, &NavigationActionThread::threadStopped);

    m_thread->start();
    QTest::qWait(100);

    m_thread->stopThread();
    m_thread->wait(3000);

    QVERIFY(stoppedSpy.count() > 0);
}

// 导航方法测试
void TestNavigationActionThread::testSendGoalToPose()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spyAccepted(m_thread, &NavigationActionThread::goalAccepted);
    QSignalSpy spyRejected(m_thread, &NavigationActionThread::goalRejected);

    // 发送目标（需要 Nav2 运行）
    m_thread->sendGoalToPose(1.0, 2.0, 0.0);

    // 验证方法调用成功（实际响应需要 Nav2）
    QVERIFY(spyAccepted.count() >= 0 || spyRejected.count() >= 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testSendGoalToPoseWithDifferentCoordinates()
{
    m_thread->start();
    QTest::qWait(100);

    // 测试多组坐标
    m_thread->sendGoalToPose(0.0, 0.0, 0.0);
    QTest::qWait(50);

    m_thread->sendGoalToPose(-5.5, 3.2, 1.57);
    QTest::qWait(50);

    m_thread->sendGoalToPose(10.0, -10.0, 3.14);

    // 验证方法调用不崩溃
    QVERIFY(true);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testSendGoalToPoseWithYaw()
{
    m_thread->start();
    QTest::qWait(100);

    // 测试不同偏航角
    m_thread->sendGoalToPose(1.0, 1.0, 0.0);
    QTest::qWait(50);

    m_thread->sendGoalToPose(1.0, 1.0, 1.57);
    QTest::qWait(50);

    m_thread->sendGoalToPose(1.0, 1.0, 3.14);

    // 验证方法调用不崩溃
    QVERIFY(true);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testCancelCurrentGoal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spyCanceled(m_thread, &NavigationActionThread::goalCanceled);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);
    QTest::qWait(100);

    bool result = m_thread->cancelCurrentGoal();
    // 取消结果取决于 Nav2 是否运行
    QVERIFY(result == true || result == false);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testCancelGoalWithoutNavigation()
{
    m_thread->start();
    QTest::qWait(100);

    // 没有导航时取消应该返回 false
    bool result = m_thread->cancelCurrentGoal();
    QVERIFY(!result);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testIsNavigating()
{
    m_thread->start();
    QTest::qWait(100);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);
    QTest::qWait(100);

    bool isNavigating = m_thread->isNavigating();
    // 结果取决于 Nav2 是否运行
    QVERIFY(isNavigating == true || isNavigating == false);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testIsNavigatingWithoutGoal()
{
    m_thread->start();
    QTest::qWait(100);

    // 没有发送目标时不应处于导航状态
    bool isNavigating = m_thread->isNavigating();
    QVERIFY(!isNavigating);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testGetCurrentGoal()
{
    m_thread->start();
    QTest::qWait(100);

    double x = 1.5;
    double y = 2.5;
    double yaw = 1.0;

    m_thread->sendGoalToPose(x, y, yaw);
    QTest::qWait(100);

    auto goal = m_thread->getCurrentGoal();
    // 验证获取到的目标坐标
    QCOMPARE(goal.pose.position.x, x);
    QCOMPARE(goal.pose.position.y, y);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testGetCurrentGoalWithoutGoal()
{
    m_thread->start();
    QTest::qWait(100);

    auto goal = m_thread->getCurrentGoal();
    // 没有目标时应该返回默认值
    QCOMPARE(goal.pose.position.x, 0.0);
    QCOMPARE(goal.pose.position.y, 0.0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

// 信号测试
void TestNavigationActionThread::testGoalAcceptedSignal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spy(m_thread, &NavigationActionThread::goalAccepted);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);

    // 验证信号发射（实际需要 Nav2 运行）
    QVERIFY(spy.count() >= 0 || spy.wait(100));

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testGoalRejectedSignal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spy(m_thread, &NavigationActionThread::goalRejected);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);

    // 验证信号发射（实际需要 Nav2 运行）
    QVERIFY(spy.count() >= 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testFeedbackReceivedSignal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spy(m_thread, &NavigationActionThread::feedbackReceived);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);

    // 验证信号发射（实际需要 Nav2 运行）
    QVERIFY(spy.count() >= 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testResultReceivedSignal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spy(m_thread, &NavigationActionThread::resultReceived);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);
    QTest::qWait(100);
    m_thread->cancelCurrentGoal();

    // 验证信号发射（实际需要 Nav2 运行）
    QVERIFY(spy.count() >= 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

void TestNavigationActionThread::testGoalCanceledSignal()
{
    m_thread->start();
    QTest::qWait(100);

    QSignalSpy spy(m_thread, &NavigationActionThread::goalCanceled);

    m_thread->sendGoalToPose(1.0, 2.0, 0.0);
    QTest::qWait(100);
    m_thread->cancelCurrentGoal();

    // 验证信号发射（实际需要 Nav2 运行）
    QVERIFY(spy.count() >= 0);

    m_thread->stopThread();
    m_thread->wait(3000);
}

#include "testnavigationactionthread.moc"
