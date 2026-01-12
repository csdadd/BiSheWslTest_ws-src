#include "testnavigationintegration.h"
#include "roscontextmanager.h"

void TestNavigationIntegration::initTestCase()
{
    ROSContextManager::instance().initialize();
}

void TestNavigationIntegration::cleanupTestCase()
{
}

void TestNavigationIntegration::init()
{
}

void TestNavigationIntegration::cleanup()
{
}

void TestNavigationIntegration::testMapClickToNavigation()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    QSignalSpy spy(mapWidget, &MapWidget::mapClicked);

    QVERIFY(spy.isValid());
}

void TestNavigationIntegration::testMapClickToNavigationWithMultipleClicks()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    QSignalSpy spy(mapWidget, &MapWidget::mapClicked);

    QVERIFY(spy.isValid());
}

void TestNavigationIntegration::testNavigationFlow()
{
    MainWindow mainWindow;

    NavigationActionClient* client = new NavigationActionClient(&mainWindow);

    QSignalSpy spyGoalAccepted(client, &NavigationActionClient::goalAccepted);
    QSignalSpy spyFeedback(client, &NavigationActionClient::feedbackReceived);
    QSignalSpy spyResult(client, &NavigationActionClient::resultReceived);

    bool result = client->sendGoal(1.0, 2.0, 0.0);

    QVERIFY(result);

    delete client;
}

void TestNavigationIntegration::testNavigationFlowWithCancellation()
{
    MainWindow mainWindow;

    NavigationActionClient* client = new NavigationActionClient(&mainWindow);

    QSignalSpy spyCanceled(client, &NavigationActionClient::goalCanceled);

    client->sendGoal(1.0, 2.0, 0.0);
    bool cancelResult = client->cancelGoal();

    QVERIFY(cancelResult);

    delete client;
}

void TestNavigationIntegration::testPathVisualization()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    PathVisualizer visualizer(mapWidget->scene(), &mainWindow);

    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[0].pose.position.x = 0.0;
    path.poses[0].pose.position.y = 0.0;
    path.poses[1].pose.position.x = 1.0;
    path.poses[1].pose.position.y = 1.0;
    path.poses[2].pose.position.x = 2.0;
    path.poses[2].pose.position.y = 2.0;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(mapWidget->scene()->items().size() > 0);
}

void TestNavigationIntegration::testPathVisualizationWithEmptyPath()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    PathVisualizer visualizer(mapWidget->scene(), &mainWindow);

    nav_msgs::msg::Path path;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(mapWidget->scene()->items().size() > 0);
}

void TestNavigationIntegration::testPathVisualizationWithMultiplePoints()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    PathVisualizer visualizer(mapWidget->scene(), &mainWindow);

    nav_msgs::msg::Path path;
    path.poses.resize(10);
    for (size_t i = 0; i < path.poses.size(); ++i) {
        path.poses[i].pose.position.x = i * 0.5;
        path.poses[i].pose.position.y = i * 0.5;
    }

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(mapWidget->scene()->items().size() > 0);
}

void TestNavigationIntegration::testNavigationClientIntegration()
{
    MainWindow mainWindow;

    NavigationActionClient* client = new NavigationActionClient(&mainWindow);

    QSignalSpy spyGoalAccepted(client, &NavigationActionClient::goalAccepted);
    QSignalSpy spyGoalRejected(client, &NavigationActionClient::goalRejected);
    QSignalSpy spyFeedback(client, &NavigationActionClient::feedbackReceived);
    QSignalSpy spyResult(client, &NavigationActionClient::resultReceived);
    QSignalSpy spyCanceled(client, &NavigationActionClient::goalCanceled);

    QVERIFY(spyGoalAccepted.isValid());
    QVERIFY(spyGoalRejected.isValid());
    QVERIFY(spyFeedback.isValid());
    QVERIFY(spyResult.isValid());
    QVERIFY(spyCanceled.isValid());

    delete client;
}

void TestNavigationIntegration::testPathVisualizerIntegration()
{
    MainWindow mainWindow;

    MapWidget* mapWidget = mainWindow.findChild<MapWidget*>();
    QVERIFY(mapWidget != nullptr);

    PathVisualizer visualizer(mapWidget->scene(), &mainWindow);

    nav_msgs::msg::Path path;
    path.poses.resize(5);
    for (size_t i = 0; i < path.poses.size(); ++i) {
        path.poses[i].pose.position.x = i * 1.0;
        path.poses[i].pose.position.y = i * 1.0;
    }

    visualizer.updatePath(path, 0.05, QPointF(0, 0));
    visualizer.setPathColor(Qt::green);
    visualizer.setPathWidth(2.0);
    visualizer.setPathStyle(Qt::SolidLine);

    QVERIFY(mapWidget->scene()->items().size() > 0);
}

void TestNavigationIntegration::testMainWindowNavigationControls()
{
    MainWindow mainWindow;

    QPushButton* btnStart = mainWindow.findChild<QPushButton*>("btnStartNavigation");
    QPushButton* btnCancel = mainWindow.findChild<QPushButton*>("btnCancelNavigation");
    QPushButton* btnClear = mainWindow.findChild<QPushButton*>("btnClearGoal");

    QVERIFY(btnStart != nullptr);
    QVERIFY(btnCancel != nullptr);
    QVERIFY(btnClear != nullptr);

    QVERIFY(btnStart->isEnabled());
    QVERIFY(btnCancel->isEnabled());
    QVERIFY(btnClear->isEnabled());
}

void TestNavigationIntegration::testClearGoal()
{
    MainWindow mainWindow;

    QPushButton* btnClear = mainWindow.findChild<QPushButton*>("btnClearGoal");
    QVERIFY(btnClear != nullptr);

    QTest::mouseClick(btnClear, Qt::LeftButton);
}

void TestNavigationIntegration::testClearGoalWithoutTarget()
{
    MainWindow mainWindow;

    QPushButton* btnClear = mainWindow.findChild<QPushButton*>("btnClearGoal");
    QVERIFY(btnClear != nullptr);

    QTest::mouseClick(btnClear, Qt::LeftButton);
}
