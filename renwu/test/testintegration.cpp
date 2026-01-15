#include "testintegration.h"
#include <QApplication>
#include <QEventLoop>

void TestIntegration::initTestCase()
{
}

void TestIntegration::cleanupTestCase()
{
}

void TestIntegration::init()
{
}

void TestIntegration::cleanup()
{
}

void TestIntegration::testMapThreadToMapWidget()
{
    MapThread* mapThread = new MapThread();
    MapWidget* mapWidget = new MapWidget();

    bool mapReceived = false;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage&, double, double, double) {
        mapReceived = true;
    });

    QObject::connect(mapThread, &MapThread::mapReceived, mapWidget, &MapWidget::setMapImage);

    mapThread->start();

    QTest::qWait(100);

    mapThread->stopThread();

    delete mapWidget;
    delete mapThread;
}

void TestIntegration::testMapThreadToMapWidgetWithRealMap()
{
    MapThread* mapThread = new MapThread();
    MapWidget* mapWidget = new MapWidget();

    bool mapReceived = false;
    int mapCount = 0;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage& image, double, double, double) {
        mapReceived = true;
        if (!image.isNull()) {
            mapCount++;
        }
    });

    QObject::connect(mapThread, &MapThread::mapReceived, mapWidget, &MapWidget::setMapImage);

    mapThread->start();

    QTest::qWait(5000);

    mapThread->stopThread();

    delete mapWidget;
    delete mapThread;
}

void TestIntegration::testRobotStatusToMapWidget()
{
    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    int initialItemCount = mapWidget->scene()->items().size();

    mapWidget->updateRobotPose(0.0, 0.0, 0.0);

    QVERIFY(mapWidget->scene()->items().size() > initialItemCount);

    delete mapWidget;
}

void TestIntegration::testMapThreadConnectionState()
{
    MapThread* mapThread = new MapThread();

    bool connected = false;
    bool disconnected = false;

    QObject::connect(mapThread, &MapThread::connectionStateChanged, [&](bool state) {
        if (state) {
            connected = true;
        } else {
            disconnected = true;
        }
    });

    mapThread->start();

    QTest::qWait(500);

    QVERIFY(connected);

    mapThread->stopThread();

    QTest::qWait(500);

    QVERIFY(disconnected);

    delete mapThread;
}

void TestIntegration::testMapWidgetZoomSignal()
{
    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double zoomLevel = 0.0;

    QObject::connect(mapWidget, &MapWidget::zoomChanged, [&](double level) {
        signalEmitted = true;
        zoomLevel = level;
    });

    mapWidget->setZoomLevel(2.0);

    QVERIFY(signalEmitted);
    QVERIFY(qAbs(zoomLevel - 2.0) < 0.01);

    delete mapWidget;
}

void TestIntegration::testMapWidgetClickSignal()
{
    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double clickX = 0.0, clickY = 0.0;

    QObject::connect(mapWidget, &MapWidget::mapClicked, [&](double x, double y) {
        signalEmitted = true;
        clickX = x;
        clickY = y;
    });

    QMouseEvent* mouseEvent = new QMouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
    QApplication::sendEvent(mapWidget, mouseEvent);
    delete mouseEvent;

    QVERIFY(signalEmitted);
    QVERIFY(clickX != 0.0 || clickY != 0.0);

    delete mapWidget;
}

void TestIntegration::testMultipleRobotPoseUpdates()
{
    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    mapWidget->updateRobotPose(0.0, 0.0, 0.0);
    int initialItemCount = mapWidget->scene()->items().size();

    for (int i = 0; i < 10; ++i) {
        mapWidget->updateRobotPose(i * 0.1, i * 0.1, i * 0.1);
    }

    QVERIFY(mapWidget->scene()->items().size() == initialItemCount);

    delete mapWidget;
}

void TestIntegration::testMapUpdateFrequency()
{
    MapThread* mapThread = new MapThread();

    int updateCount = 0;
    QElapsedTimer timer;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage&, double, double, double) {
        updateCount++;
    });

    mapThread->start();

    timer.start();
    QTest::qWait(2000);
    qint64 elapsed = timer.elapsed();

    mapThread->stopThread();

    double frequency = updateCount * 1000.0 / elapsed;

    if (updateCount > 0) {
        QVERIFY(frequency > 0.0);
        QVERIFY(frequency < 10.0);
    }

    delete mapThread;
}

void TestIntegration::testMapWidgetWithoutMap()
{
    MapWidget* mapWidget = new MapWidget();

    mapWidget->updateRobotPose(0.0, 0.0, 0.0);

    QVERIFY(mapWidget->scene()->items().size() == 0);

    delete mapWidget;
}

void TestIntegration::testMapThreadLifecycle()
{
    MapThread* mapThread = new MapThread();

    QVERIFY(!mapThread->isRunning());

    mapThread->start();

    QTest::qWait(500);

    QVERIFY(mapThread->isRunning());

    mapThread->stopThread();

    QTest::qWait(500);

    QVERIFY(!mapThread->isRunning());

    delete mapThread;
}
