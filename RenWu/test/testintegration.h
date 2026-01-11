#ifndef TESTINTEGRATION_H
#define TESTINTEGRATION_H

#include <QtTest/QtTest>
#include <QImage>
#include <QTimer>
#include "mapthread.h"
#include "mapwidget.h"
#include "robotstatusthread.h"

class TestIntegration : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testMapThreadToMapWidget();
    void testMapThreadToMapWidgetWithRealMap();
    void testRobotStatusToMapWidget();
    void testMapThreadConnectionState();
    void testMapWidgetZoomSignal();
    void testMapWidgetClickSignal();
    void testMultipleRobotPoseUpdates();
    void testMapUpdateFrequency();
    void testMapWidgetWithoutMap();
    void testMapThreadLifecycle();
};

#endif
