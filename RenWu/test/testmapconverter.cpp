#include "testmapconverter.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <QtMath>

void TestMapConverter::initTestCase()
{
}

void TestMapConverter::cleanupTestCase()
{
}

void TestMapConverter::init()
{
}

void TestMapConverter::cleanup()
{
}

void TestMapConverter::testConvertToImage()
{
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width = 100;
    map->info.height = 100;
    map->info.resolution = 0.05;
    map->data.resize(100 * 100, 0);

    QImage image = MapConverter::convertToImage(map);

    QVERIFY(!image.isNull());
    QVERIFY(image.width() == 100);
    QVERIFY(image.height() == 100);
}

void TestMapConverter::testConvertToImageWithNullMap()
{
    nav_msgs::msg::OccupancyGrid::SharedPtr map = nullptr;

    QImage image = MapConverter::convertToImage(map);

    QVERIFY(image.isNull());
}

void TestMapConverter::testConvertToImageWithEmptyData()
{
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width = 100;
    map->info.height = 100;
    map->info.resolution = 0.05;
    map->data.clear();

    QImage image = MapConverter::convertToImage(map);

    QVERIFY(!image.isNull());
    QVERIFY(image.width() == 100);
    QVERIFY(image.height() == 100);
}

void TestMapConverter::testConvertToImageWithDifferentValues()
{
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width = 10;
    map->info.height = 10;
    map->info.resolution = 0.05;
    map->data.resize(100);

    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            int index = i * 10 + j;
            if (index < 10) {
                map->data[index] = -1;
            } else if (index < 20) {
                map->data[index] = 0;
            } else if (index < 30) {
                map->data[index] = 100;
            } else {
                map->data[index] = 50;
            }
        }
    }

    QImage image = MapConverter::convertToImage(map);

    QVERIFY(!image.isNull());
    QVERIFY(image.width() == 10);
    QVERIFY(image.height() == 10);
}

void TestMapConverter::testCoordinateConversion()
{
    double resolution = 0.05;
    double originX = -5.0;
    double originY = -5.0;

    QPointF imagePos = MapConverter::mapToImage(0.0, 0.0, resolution, originX, originY);
    QPointF mapPos = MapConverter::imageToMap(static_cast<int>(imagePos.x()),
                                                static_cast<int>(imagePos.y()),
                                                resolution, originX, originY);

    QVERIFY(qAbs(mapPos.x() - 0.0) < 0.001);
    QVERIFY(qAbs(mapPos.y() - 0.0) < 0.001);
}

void TestMapConverter::testCoordinateConversionRoundTrip()
{
    double resolution = 0.05;
    double originX = -5.0;
    double originY = -5.0;

    for (int i = -10; i <= 10; i += 2) {
        for (int j = -10; j <= 10; j += 2) {
            double mapX = i * 0.5;
            double mapY = j * 0.5;

            QPointF imagePos = MapConverter::mapToImage(mapX, mapY, resolution, originX, originY);
            QPointF mapPos = MapConverter::imageToMap(static_cast<int>(imagePos.x()),
                                                        static_cast<int>(imagePos.y()),
                                                        resolution, originX, originY);

            QVERIFY(qAbs(mapPos.x() - mapX) < 0.1);
            QVERIFY(qAbs(mapPos.y() - mapY) < 0.1);
        }
    }
}

void TestMapConverter::testCoordinateConversionWithInvalidResolution()
{
    double resolution = 0.0;
    double originX = -5.0;
    double originY = -5.0;

    QPointF imagePos = MapConverter::mapToImage(0.0, 0.0, resolution, originX, originY);

    QVERIFY(qIsInf(imagePos.x()) || qIsNaN(imagePos.x()));
}

void TestMapConverter::testCreateRobotPolygon()
{
    QPolygonF polygon = MapConverter::createRobotPolygon(1.0, 2.0, M_PI / 4, 0.5);

    QVERIFY(polygon.size() == 3);

    QPointF center = polygon.boundingRect().center();
    QVERIFY(qAbs(center.x() - 1.0) < 0.01);
    QVERIFY(qAbs(center.y() - 2.0) < 0.01);
}

void TestMapConverter::testCreateRobotPolygonWithDifferentAngles()
{
    for (int i = 0; i < 360; i += 45) {
        double angle = qDegreesToRadians(static_cast<double>(i));
        QPolygonF polygon = MapConverter::createRobotPolygon(0.0, 0.0, angle, 1.0);

        QVERIFY(polygon.size() == 3);

        QPointF center = polygon.boundingRect().center();
        QVERIFY(qAbs(center.x()) < 0.01);
        QVERIFY(qAbs(center.y()) < 0.01);
    }
}

void TestMapConverter::testCreateRobotPolygonWithDifferentSizes()
{
    for (int i = 1; i <= 10; ++i) {
        double size = i * 0.1;
        QPolygonF polygon = MapConverter::createRobotPolygon(0.0, 0.0, 0.0, size);

        QVERIFY(polygon.size() == 3);

        QRectF boundingRect = polygon.boundingRect();
        QVERIFY(boundingRect.width() > 0);
        QVERIFY(boundingRect.height() > 0);
    }
}

void TestMapConverter::testCreateRobotPolygonCenter()
{
    QPolygonF polygon = MapConverter::createRobotPolygon(5.0, 3.0, M_PI / 2, 1.0);

    QVERIFY(polygon.size() == 3);

    QPointF center = polygon.boundingRect().center();
    QVERIFY(qAbs(center.x() - 5.0) < 0.01);
    QVERIFY(qAbs(center.y() - 3.0) < 0.01);
}
