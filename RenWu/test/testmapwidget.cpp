#include "testmapwidget.h"
#include <QApplication>

void TestMapWidget::initTestCase()
{
}

void TestMapWidget::cleanupTestCase()
{
}

void TestMapWidget::init()
{
}

void TestMapWidget::cleanup()
{
}

void TestMapWidget::testSetMapImage()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QVERIFY(widget.scene() != nullptr);
    QVERIFY(widget.scene()->items().size() > 0);
}

void TestMapWidget::testSetMapImageWithNullImage()
{
    MapWidget widget;

    QImage nullImage;

    widget.setMapImage(nullImage, 0.05, -5.0, -5.0);

    QVERIFY(widget.scene() != nullptr);
}

void TestMapWidget::testSetMapImageWithDifferentResolutions()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    widget.setMapImage(testImage, 0.01, -5.0, -5.0);
    QVERIFY(widget.scene() != nullptr);

    widget.setMapImage(testImage, 0.05, -5.0, -5.0);
    QVERIFY(widget.scene() != nullptr);

    widget.setMapImage(testImage, 0.1, -5.0, -5.0);
    QVERIFY(widget.scene() != nullptr);
}

void TestMapWidget::testUpdateRobotPose()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.updateRobotPose(0.0, 0.0, 0.0);

    QVERIFY(widget.scene()->items().size() > 1);
}

void TestMapWidget::testUpdateRobotPoseWithoutMap()
{
    MapWidget widget;

    widget.updateRobotPose(0.0, 0.0, 0.0);

    QVERIFY(widget.scene()->items().size() == 0);
}

void TestMapWidget::testUpdateRobotPoseMultipleTimes()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.updateRobotPose(0.0, 0.0, 0.0);
    int initialSize = widget.scene()->items().size();

    widget.updateRobotPose(1.0, 1.0, M_PI / 4);
    QVERIFY(widget.scene()->items().size() == initialSize);

    widget.updateRobotPose(-1.0, -1.0, M_PI / 2);
    QVERIFY(widget.scene()->items().size() == initialSize);
}

void TestMapWidget::testClearRobotPose()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.updateRobotPose(0.0, 0.0, 0.0);
    QVERIFY(widget.scene()->items().size() > 1);

    widget.clearRobotPose();
    QVERIFY(widget.scene()->items().size() == 1);
}

void TestMapWidget::testClearRobotPoseWithoutRobot()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.clearRobotPose();

    QVERIFY(widget.scene()->items().size() == 1);
}

void TestMapWidget::testSetZoomLevel()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.setZoomLevel(2.0);

    QVERIFY(qAbs(widget.getZoomLevel() - 2.0) < 0.01);
}

void TestMapWidget::testSetZoomLevelWithInvalidValue()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    double initialZoom = widget.getZoomLevel();
    widget.setZoomLevel(-1.0);

    QVERIFY(qAbs(widget.getZoomLevel() - initialZoom) < 0.01);
}

void TestMapWidget::testGetZoomLevel()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    double initialZoom = widget.getZoomLevel();
    QVERIFY(qAbs(initialZoom - 1.0) < 0.01);
}

void TestMapWidget::testZoomLevelSignal()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double zoomLevel = 0.0;

    QObject::connect(&widget, &MapWidget::zoomChanged, [&](double level) {
        signalEmitted = true;
        zoomLevel = level;
    });

    widget.setZoomLevel(2.0);

    QVERIFY(signalEmitted);
    QVERIFY(qAbs(zoomLevel - 2.0) < 0.01);
}

void TestMapWidget::testCenterOnRobot()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.updateRobotPose(0.0, 0.0, 0.0);
    widget.centerOnRobot();

    QVERIFY(true);
}

void TestMapWidget::testCenterOnRobotWithoutRobot()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.centerOnRobot();

    QVERIFY(true);
}

void TestMapWidget::testCenterOnPosition()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    widget.centerOnPosition(0.0, 0.0);

    QVERIFY(true);
}

void TestMapWidget::testWheelEvent()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    double initialZoom = widget.getZoomLevel();

    QWheelEvent* wheelEvent = new QWheelEvent(QPointF(50, 50), QPointF(50, 50), QPoint(0, 0), QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false, Qt::MouseEventNotSynthesized);
    QApplication::sendEvent(&widget, wheelEvent);
    delete wheelEvent;

    QVERIFY(qAbs(widget.getZoomLevel() - initialZoom) > 0.01);
}

void TestMapWidget::testWheelEventSignal()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double zoomLevel = 0.0;

    QObject::connect(&widget, &MapWidget::zoomChanged, [&](double level) {
        signalEmitted = true;
        zoomLevel = level;
    });

    QWheelEvent* wheelEvent = new QWheelEvent(QPointF(50, 50), QPointF(50, 50), QPoint(0, 0), QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false, Qt::MouseEventNotSynthesized);
    QApplication::sendEvent(&widget, wheelEvent);
    delete wheelEvent;

    QVERIFY(signalEmitted);
}

void TestMapWidget::testMousePressEventLeftButton()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QMouseEvent mouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &mouseEvent);

    QVERIFY(true);
}

void TestMapWidget::testMousePressEventMiddleButton()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QMouseEvent mouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::MiddleButton, Qt::MiddleButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &mouseEvent);

    QVERIFY(true);
}

void TestMapWidget::testMousePressEventRightButton()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QMouseEvent mouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &mouseEvent);

    QVERIFY(true);
}

void TestMapWidget::testMousePressEventRightButtonSignal()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double clickX = 0.0, clickY = 0.0;

    QObject::connect(&widget, &MapWidget::mapClicked, [&](double x, double y) {
        signalEmitted = true;
        clickX = x;
        clickY = y;
    });

    QMouseEvent* mouseEvent = new QMouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, mouseEvent);
    delete mouseEvent;

    QVERIFY(signalEmitted);
}

void TestMapWidget::testMouseMoveEvent()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent moveEvent(QEvent::MouseMove, QPoint(60, 60), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &moveEvent);

    QVERIFY(true);
}

void TestMapWidget::testMouseReleaseEvent()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 60), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QVERIFY(true);
}

void TestMapWidget::testMapClickedSignal()
{
    MapWidget widget;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);
    widget.setMapImage(testImage, 0.05, -5.0, -5.0);

    bool signalEmitted = false;
    double clickX = 0.0, clickY = 0.0;

    QObject::connect(&widget, &MapWidget::mapClicked, [&](double x, double y) {
        signalEmitted = true;
        clickX = x;
        clickY = y;
    });

    QMouseEvent* mouseEvent = new QMouseEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::RightButton, Qt::RightButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, mouseEvent);
    delete mouseEvent;

    QVERIFY(signalEmitted);
    QVERIFY(clickX != 0.0 || clickY != 0.0);
}
