#include "testmaploader.h"
#include <QFile>
#include <QDir>
#include <QDebug>
#include <QtMath>

void TestMapLoader::initTestCase()
{
    m_testDataDir = QDir::tempPath() + "/test_maploader";
    QDir dir;
    dir.mkpath(m_testDataDir);

    QString yamlContent = R"(
image: test_map.pgm
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
)";

    QFile yamlFile(m_testDataDir + "/test_map.yaml");
    if (yamlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&yamlFile);
        out << yamlContent;
        yamlFile.close();
    }

    QFile pgmFile(m_testDataDir + "/test_map.pgm");
    if (pgmFile.open(QIODevice::WriteOnly)) {
        QByteArray pgmData;
        pgmData.append("P5\n");
        pgmData.append("10 10\n");
        pgmData.append("255\n");
        for (int i = 0; i < 100; i++) {
            pgmData.append(static_cast<char>(i % 256));
        }
        pgmFile.write(pgmData);
        pgmFile.close();
    }
}

void TestMapLoader::cleanupTestCase()
{
    QDir dir(m_testDataDir);
    dir.removeRecursively();
}

void TestMapLoader::init()
{
}

void TestMapLoader::cleanup()
{
}

void TestMapLoader::testParseYaml()
{
    QString yamlPath = m_testDataDir + "/test_map.yaml";
    QMap<QString, QVariant> metaData = MapLoader::parseYaml(yamlPath);

    QVERIFY(metaData.contains("image"));
    QVERIFY(metaData.contains("resolution"));
    QVERIFY(metaData.contains("origin"));

    QCOMPARE(metaData["image"].toString(), QString("test_map.pgm"));
    QCOMPARE(metaData["resolution"].toDouble(), 0.05);

    QList<QVariant> originList = metaData["origin"].toList();
    QCOMPARE(originList.size(), 3);
    QCOMPARE(originList[0].toDouble(), -5.0);
    QCOMPARE(originList[1].toDouble(), -5.0);
    QCOMPARE(originList[2].toDouble(), 0.0);
}

void TestMapLoader::testLoadPgm()
{
    QString pgmPath = m_testDataDir + "/test_map.pgm";
    QImage image = MapLoader::loadPgm(pgmPath);

    QVERIFY(!image.isNull());
    QCOMPARE(image.width(), 10);
    QCOMPARE(image.height(), 10);
}

void TestMapLoader::testLoadFromFile()
{
    QString yamlPath = m_testDataDir + "/test_map.yaml";
    auto map = MapLoader::loadFromFile(yamlPath);

    QVERIFY(map != nullptr);
    QCOMPARE(map->info.width, 10);
    QCOMPARE(map->info.height, 10);
    QVERIFY(qAbs(map->info.resolution - 0.05) < 0.001);
    QCOMPARE(map->info.origin.position.x, -5.0);
    QCOMPARE(map->info.origin.position.y, -5.0);
    QCOMPARE(map->info.origin.position.z, 0.0);

    QCOMPARE(map->data.size(), 100);
}

void TestMapLoader::testMapCache()
{
    MapCache cache(3);

    QString yamlPath = m_testDataDir + "/test_map.yaml";
    auto map = MapLoader::loadFromFile(yamlPath);

    QVERIFY(!cache.contains("test_key"));

    cache.add("test_key", map);
    QVERIFY(cache.contains("test_key"));

    auto cachedMap = cache.get("test_key");
    QVERIFY(cachedMap != nullptr);
    QCOMPARE(cachedMap->info.width, map->info.width);

    QCOMPARE(cache.size(), 1);

    cache.remove("test_key");
    QVERIFY(!cache.contains("test_key"));
    QCOMPARE(cache.size(), 0);
}

void TestMapLoader::testMapCacheEviction()
{
    MapCache cache(2);

    QString yamlPath = m_testDataDir + "/test_map.yaml";
    auto map1 = MapLoader::loadFromFile(yamlPath);
    auto map2 = MapLoader::loadFromFile(yamlPath);
    auto map3 = MapLoader::loadFromFile(yamlPath);

    cache.add("key1", map1);
    QCOMPARE(cache.size(), 1);

    cache.add("key2", map2);
    QCOMPARE(cache.size(), 2);

    cache.add("key3", map3);
    QCOMPARE(cache.size(), 2);

    QVERIFY(!cache.contains("key1"));
    QVERIFY(cache.contains("key2"));
    QVERIFY(cache.contains("key3"));
}
