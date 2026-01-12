#include "testmapmarker.h"

void TestMapMarker::initTestCase()
{
}

void TestMapMarker::cleanupTestCase()
{
}

void TestMapMarker::init()
{
    m_manager = new MapMarkerManager();
}

void TestMapMarker::cleanup()
{
    delete m_manager;
    m_manager = nullptr;
}

void TestMapMarker::testMarkerDefaultConstructor()
{
    MapMarker marker;
    QVERIFY(marker.name.isEmpty());
    QVERIFY(marker.position.isNull());
    QVERIFY(marker.description.isEmpty());
}

void TestMapMarker::testMarkerParameterizedConstructor()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Test Description");
    QCOMPARE(marker.name, QString("Test"));
    QCOMPARE(marker.position.x(), 1.0);
    QCOMPARE(marker.position.y(), 2.0);
    QCOMPARE(marker.color, Qt::red);
    QCOMPARE(marker.description, QString("Test Description"));
}

void TestMapMarker::testMarkerManagerConstructor()
{
    QVERIFY(m_manager != nullptr);
    QVERIFY(m_manager->getMarkers().isEmpty());
}

void TestMapMarker::testAddMarker()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    m_manager->addMarker(marker);
    QCOMPARE(m_manager->getMarkers().size(), 1);
    QCOMPARE(m_manager->getMarkers().first().name, QString("Test"));
}

void TestMapMarker::testRemoveMarker()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    m_manager->addMarker(marker);
    QCOMPARE(m_manager->getMarkers().size(), 1);

    m_manager->removeMarker("Test");
    QCOMPARE(m_manager->getMarkers().size(), 0);
}

void TestMapMarker::testGetMarkers()
{
    MapMarker marker1(1.0, 2.0, Qt::red, "Test1", "Description1");
    MapMarker marker2(3.0, 4.0, Qt::blue, "Test2", "Description2");
    m_manager->addMarker(marker1);
    m_manager->addMarker(marker2);

    QList<MapMarker> markers = m_manager->getMarkers();
    QCOMPARE(markers.size(), 2);
}

void TestMapMarker::testClear()
{
    MapMarker marker1(1.0, 2.0, Qt::red, "Test1", "Description1");
    MapMarker marker2(3.0, 4.0, Qt::blue, "Test2", "Description2");
    m_manager->addMarker(marker1);
    m_manager->addMarker(marker2);
    QCOMPARE(m_manager->getMarkers().size(), 2);

    m_manager->clear();
    QCOMPARE(m_manager->getMarkers().size(), 0);
}

void TestMapMarker::testContains()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    QVERIFY(!m_manager->contains("Test"));

    m_manager->addMarker(marker);
    QVERIFY(m_manager->contains("Test"));
}

void TestMapMarker::testGetMarker()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    m_manager->addMarker(marker);

    MapMarker retrieved = m_manager->getMarker("Test");
    QCOMPARE(retrieved.name, QString("Test"));
    QCOMPARE(retrieved.position.x(), 1.0);
    QCOMPARE(retrieved.position.y(), 2.0);
}

void TestMapMarker::testMarkerPosition()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    QCOMPARE(marker.position.x(), 1.0);
    QCOMPARE(marker.position.y(), 2.0);
}

void TestMapMarker::testMarkerColor()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    QCOMPARE(marker.color, Qt::red);
}

void TestMapMarker::testMarkerDescription()
{
    MapMarker marker(1.0, 2.0, Qt::red, "Test", "Description");
    QCOMPARE(marker.description, QString("Description"));
}
