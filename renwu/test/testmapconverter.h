#ifndef TESTMAPCONVERTER_H
#define TESTMAPCONVERTER_H

#include <QtTest/QtTest>
#include <QImage>
#include <QPointF>
#include <QPolygonF>
#include <cmath>
#include "mapconverter.h"

class TestMapConverter : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConvertToImage();
    void testConvertToImageWithNullMap();
    void testConvertToImageWithEmptyData();
    void testConvertToImageWithDifferentValues();
    void testCoordinateConversion();
    void testCoordinateConversionRoundTrip();
    void testCoordinateConversionWithInvalidResolution();
    void testCreateRobotPolygon();
    void testCreateRobotPolygonWithDifferentAngles();
    void testCreateRobotPolygonWithDifferentSizes();
    void testCreateRobotPolygonCenter();
};

#endif
