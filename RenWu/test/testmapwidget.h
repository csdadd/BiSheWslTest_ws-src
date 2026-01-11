#ifndef TESTMAPWIDGET_H
#define TESTMAPWIDGET_H

#include <QtTest/QtTest>
#include <QImage>
#include <QMouseEvent>
#include <QtMath>
#include "mapwidget.h"

class TestMapWidget : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testSetMapImage();
    void testSetMapImageWithNullImage();
    void testSetMapImageWithDifferentResolutions();
    void testUpdateRobotPose();
    void testUpdateRobotPoseWithoutMap();
    void testUpdateRobotPoseMultipleTimes();
    void testClearRobotPose();
    void testClearRobotPoseWithoutRobot();
    void testSetZoomLevel();
    void testSetZoomLevelWithInvalidValue();
    void testGetZoomLevel();
    void testZoomLevelSignal();
    void testCenterOnRobot();
    void testCenterOnRobotWithoutRobot();
    void testCenterOnPosition();
    void testWheelEvent();
    void testWheelEventSignal();
    void testMousePressEventLeftButton();
    void testMousePressEventMiddleButton();
    void testMousePressEventRightButton();
    void testMousePressEventRightButtonSignal();
    void testMouseMoveEvent();
    void testMouseReleaseEvent();
    void testMapClickedSignal();
};

#endif
