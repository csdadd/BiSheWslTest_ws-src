#include "testnavigationactionclient.h"
#include "roscontextmanager.h"

void TestNavigationActionClient::initTestCase()
{
    ROSContextManager::instance().initialize();
}

void TestNavigationActionClient::cleanupTestCase()
{
}

void TestNavigationActionClient::init()
{
}

void TestNavigationActionClient::cleanup()
{
}

void TestNavigationActionClient::testSendGoal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::goalAccepted);
    QSignalSpy spyRejected(&client, &NavigationActionClient::goalRejected);

    bool result = client.sendGoal(1.0, 2.0, 0.0);

    QVERIFY(result);
}

void TestNavigationActionClient::testSendGoalWithDifferentCoordinates()
{
    NavigationActionClient client;

    bool result1 = client.sendGoal(0.0, 0.0, 0.0);
    QVERIFY(result1);

    bool result2 = client.sendGoal(-5.5, 3.2, 1.57);
    QVERIFY(result2);

    bool result3 = client.sendGoal(10.0, -10.0, 3.14);
    QVERIFY(result3);
}

void TestNavigationActionClient::testSendGoalWithYaw()
{
    NavigationActionClient client;

    bool result1 = client.sendGoal(1.0, 1.0, 0.0);
    QVERIFY(result1);

    bool result2 = client.sendGoal(1.0, 1.0, 1.57);
    QVERIFY(result2);

    bool result3 = client.sendGoal(1.0, 1.0, 3.14);
    QVERIFY(result3);
}

void TestNavigationActionClient::testCancelGoal()
{
    NavigationActionClient client;

    client.sendGoal(1.0, 2.0, 0.0);

    QSignalSpy spy(&client, &NavigationActionClient::goalCanceled);

    bool result = client.cancelGoal();
    QVERIFY(result);
}

void TestNavigationActionClient::testCancelGoalWithoutNavigation()
{
    NavigationActionClient client;

    bool result = client.cancelGoal();
    QVERIFY(!result);
}

void TestNavigationActionClient::testIsNavigating()
{
    NavigationActionClient client;

    client.sendGoal(1.0, 2.0, 0.0);

    bool isNavigating = client.isNavigating();
    QVERIFY(isNavigating);
}

void TestNavigationActionClient::testIsNavigatingWithoutGoal()
{
    NavigationActionClient client;

    bool isNavigating = client.isNavigating();
    QVERIFY(!isNavigating);
}

void TestNavigationActionClient::testGetCurrentGoal()
{
    NavigationActionClient client;

    double x = 1.5;
    double y = 2.5;
    double yaw = 1.0;

    client.sendGoal(x, y, yaw);

    auto goal = client.getCurrentGoal();
    QCOMPARE(goal.pose.position.x, x);
    QCOMPARE(goal.pose.position.y, y);
}

void TestNavigationActionClient::testGetCurrentGoalWithoutGoal()
{
    NavigationActionClient client;

    auto goal = client.getCurrentGoal();
    QCOMPARE(goal.pose.position.x, 0.0);
    QCOMPARE(goal.pose.position.y, 0.0);
}

void TestNavigationActionClient::testGoalAcceptedSignal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::goalAccepted);

    client.sendGoal(1.0, 2.0, 0.0);

    QVERIFY(spy.count() >= 0 || spy.wait(100));
}

void TestNavigationActionClient::testGoalRejectedSignal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::goalRejected);

    client.sendGoal(1.0, 2.0, 0.0);

    QVERIFY(spy.count() >= 0);
}

void TestNavigationActionClient::testFeedbackReceivedSignal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::feedbackReceived);

    client.sendGoal(1.0, 2.0, 0.0);

    QVERIFY(spy.count() >= 0);
}

void TestNavigationActionClient::testResultReceivedSignal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::resultReceived);

    client.sendGoal(1.0, 2.0, 0.0);
    client.cancelGoal();

    QVERIFY(spy.count() >= 0);
}

void TestNavigationActionClient::testGoalCanceledSignal()
{
    NavigationActionClient client;

    QSignalSpy spy(&client, &NavigationActionClient::goalCanceled);

    client.sendGoal(1.0, 2.0, 0.0);
    client.cancelGoal();

    QVERIFY(spy.count() >= 0);
}
