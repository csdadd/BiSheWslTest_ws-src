#ifndef TESTNAVIGATIONINTEGRATION_H
#define TESTNAVIGATIONINTEGRATION_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QTimer>
#include <QPushButton>
#include "mainwindow.h"
#include "navigationactionclient.h"
#include "pathvisualizer.h"
#include "mapwidget.h"

class TestNavigationIntegration : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testMapClickToNavigation();
    void testMapClickToNavigationWithMultipleClicks();
    void testNavigationFlow();
    void testNavigationFlowWithCancellation();
    void testPathVisualization();
    void testPathVisualizationWithEmptyPath();
    void testPathVisualizationWithMultiplePoints();
    void testNavigationClientIntegration();
    void testPathVisualizerIntegration();
    void testMainWindowNavigationControls();
    void testClearGoal();
    void testClearGoalWithoutTarget();
};

#endif
