#ifndef TESTSYSTEM_H
#define TESTSYSTEM_H

#include <QtTest/QtTest>
#include "../mapthread.h"
#include "../mapwidget.h"

class TestSystem : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testMapDisplayInGazebo();
    void testRobotPoseVisualization();
    void testMapZoomAndDrag();
    void testMapClickFunction();
    void testLongRunStability();
    void testLargeMapPerformance();
    void testHighFrequencyMapUpdate();
};

#endif
