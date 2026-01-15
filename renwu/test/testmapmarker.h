#ifndef TESTMAPMARKER_H
#define TESTMAPMARKER_H

#include <QtTest/QtTest>
#include <QColor>
#include <QPointF>
#include "mapmarker.h"

class TestMapMarker : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testMarkerDefaultConstructor();
    void testMarkerParameterizedConstructor();
    void testMarkerManagerConstructor();
    void testAddMarker();
    void testRemoveMarker();
    void testGetMarkers();
    void testClear();
    void testContains();
    void testGetMarker();
    void testMarkerPosition();
    void testMarkerColor();
    void testMarkerDescription();

private:
    MapMarkerManager* m_manager;
};

#endif
