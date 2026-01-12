#ifndef TESTPATHVISUALIZER_H
#define TESTPATHVISUALIZER_H

#include <QtTest/QtTest>
#include <QGraphicsScene>
#include <nav_msgs/msg/path.hpp>
#include "pathvisualizer.h"

class TestPathVisualizer : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testUpdatePath();
    void testUpdatePathWithEmptyPath();
    void testUpdatePathWithSinglePoint();
    void testUpdatePathWithMultiplePoints();
    void testClearPath();
    void testClearPathAfterUpdate();
    void testSetPathColor();
    void testSetPathColorWithDifferentColors();
    void testSetPathWidth();
    void testSetPathWidthWithDifferentValues();
    void testSetPathStyle();
    void testSetPathStyleWithDifferentStyles();
    void testMapToScene();
    void testMapToSceneWithDifferentOrigins();
};

#endif
