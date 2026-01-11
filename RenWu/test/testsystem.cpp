#include "testsystem.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QTimer>
#include <QDebug>

void TestSystem::initTestCase()
{
    qDebug() << "[TestSystem] 初始化系统测试";
}

void TestSystem::cleanupTestCase()
{
    qDebug() << "[TestSystem] 清理系统测试";
}

void TestSystem::init()
{
}

void TestSystem::cleanup()
{
}

void TestSystem::testMapDisplayInGazebo()
{
    qDebug() << "[TestSystem] 测试地图在 Gazebo 仿真环境中的显示";

    MapThread* mapThread = new MapThread();
    MapWidget* mapWidget = new MapWidget();

    bool mapReceived = false;
    bool mapSet = false;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage& image, double, double, double) {
        mapReceived = true;
        if (!image.isNull()) {
            qDebug() << "[TestSystem] 地图已接收 - 尺寸:" << image.width() << "x" << image.height();
        }
    });

    QObject::connect(mapThread, &MapThread::mapReceived, mapWidget, [&](const QImage& image, double resolution, double originX, double originY) {
        if (!image.isNull()) {
            mapWidget->setMapImage(image, resolution, originX, originY);
            mapSet = true;
            qDebug() << "[TestSystem] 地图已设置 - 分辨率:" << resolution << "原点:(" << originX << "," << originY << ")";
        }
    });

    mapThread->start();

    QTest::qWait(5000);

    mapThread->stopThread();

    delete mapWidget;
    delete mapThread;

    qDebug() << "[TestSystem] 地图显示测试完成";
}

void TestSystem::testRobotPoseVisualization()
{
    qDebug() << "[TestSystem] 测试机器人位姿可视化";

    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    mapWidget->updateRobotPose(0.0, 0.0, 0.0);
    QTest::qWait(100);

    mapWidget->updateRobotPose(1.0, 1.0, M_PI / 4);
    QTest::qWait(100);

    mapWidget->updateRobotPose(-1.0, -1.0, M_PI / 2);
    QTest::qWait(100);

    QVERIFY(mapWidget->scene()->items().size() > 1);

    delete mapWidget;

    qDebug() << "[TestSystem] 机器人位姿可视化测试完成";
}

void TestSystem::testMapZoomAndDrag()
{
    qDebug() << "[TestSystem] 测试地图缩放和拖拽";

    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    double initialZoom = mapWidget->getZoomLevel();

    mapWidget->setZoomLevel(2.0);
    QTest::qWait(100);

    QVERIFY(qAbs(mapWidget->getZoomLevel() - 2.0) < 0.01);

    mapWidget->setZoomLevel(0.5);
    QTest::qWait(100);

    QVERIFY(qAbs(mapWidget->getZoomLevel() - 0.5) < 0.01);

    mapWidget->setZoomLevel(1.0);
    QTest::qWait(100);

    QVERIFY(qAbs(mapWidget->getZoomLevel() - 1.0) < 0.01);

    delete mapWidget;

    qDebug() << "[TestSystem] 地图缩放和拖拽测试完成";
}

void TestSystem::testMapClickFunction()
{
    qDebug() << "[TestSystem] 测试地图点击功能";

    MapWidget* mapWidget = new MapWidget();

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    mapWidget->setMapImage(testImage, 0.05, -5.0, -5.0);

    bool clicked = false;
    double clickX = 0.0, clickY = 0.0;

    QObject::connect(mapWidget, &MapWidget::mapClicked, [&](double x, double y) {
        clicked = true;
        clickX = x;
        clickY = y;
        qDebug() << "[TestSystem] 地图点击 - 坐标:(" << x << "," << y << ")";
    });

    QMouseEvent* mouseEvent = new QMouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
    QApplication::sendEvent(mapWidget, mouseEvent);
    delete mouseEvent;

    QTest::qWait(100);

    delete mapWidget;

    qDebug() << "[TestSystem] 地图点击功能测试完成";
}

void TestSystem::testLongRunStability()
{
    qDebug() << "[TestSystem] 测试长时间运行稳定性（10秒）";

    MapThread* mapThread = new MapThread();
    MapWidget* mapWidget = new MapWidget();

    int updateCount = 0;
    QElapsedTimer timer;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage&, double, double, double) {
        updateCount++;
    });

    QObject::connect(mapThread, &MapThread::mapReceived, mapWidget, &MapWidget::setMapImage);

    mapThread->start();

    timer.start();
    QTest::qWait(10000);
    qint64 elapsed = timer.elapsed();

    mapThread->stopThread();

    qDebug() << "[TestSystem] 长时间运行测试完成 - 运行时间:" << elapsed << "ms 地图更新次数:" << updateCount;

    delete mapWidget;
    delete mapThread;
}

void TestSystem::testLargeMapPerformance()
{
    qDebug() << "[TestSystem] 测试大地图（1024x1024）显示性能";

    MapWidget* mapWidget = new MapWidget();

    QImage largeMap(1024, 1024, QImage::Format_RGB32);
    largeMap.fill(Qt::white);

    QElapsedTimer timer;
    timer.start();

    mapWidget->setMapImage(largeMap, 0.05, -25.6, -25.6);

    qint64 elapsed = timer.elapsed();

    qDebug() << "[TestSystem] 大地图设置耗时:" << elapsed << "ms";

    QVERIFY(elapsed < 1000);

    delete mapWidget;

    qDebug() << "[TestSystem] 大地图性能测试完成";
}

void TestSystem::testHighFrequencyMapUpdate()
{
    qDebug() << "[TestSystem] 测试高频地图更新性能";

    MapThread* mapThread = new MapThread();

    int updateCount = 0;
    QElapsedTimer timer;

    QObject::connect(mapThread, &MapThread::mapReceived, [&](const QImage&, double, double, double) {
        updateCount++;
    });

    mapThread->start();

    timer.start();
    QTest::qWait(5000);
    qint64 elapsed = timer.elapsed();

    mapThread->stopThread();

    double frequency = updateCount * 1000.0 / elapsed;

    qDebug() << "[TestSystem] 高频更新测试完成 - 更新次数:" << updateCount << "频率:" << frequency << "Hz";

    if (updateCount > 0) {
        QVERIFY(frequency > 0.0);
        QVERIFY(frequency < 10.0);
    }

    delete mapThread;

    qDebug() << "[TestSystem] 高频地图更新测试完成";
}
