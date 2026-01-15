#include "testmapcache.h"

void TestMapCache::initTestCase()
{
}

void TestMapCache::cleanupTestCase()
{
}

void TestMapCache::init()
{
    m_cache = new MapCache(10);
}

void TestMapCache::cleanup()
{
    delete m_cache;
    m_cache = nullptr;
}

void TestMapCache::testConstructor()
{
    QVERIFY(m_cache != nullptr);
    QCOMPARE(m_cache->maxSize(), 10);
    QCOMPARE(m_cache->size(), 0);
}

void TestMapCache::testAdd()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto map2 = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    m_cache->add("map1", map1);
    QCOMPARE(m_cache->size(), 1);

    m_cache->add("map2", map2);
    QCOMPARE(m_cache->size(), 2);
}

void TestMapCache::testGet()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map1->header.frame_id = "test_frame";

    m_cache->add("map1", map1);

    auto retrieved = m_cache->get("map1");
    QVERIFY(retrieved != nullptr);
    QCOMPARE(QString::fromStdString(retrieved->header.frame_id), QString("test_frame"));
}

void TestMapCache::testRemove()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    m_cache->add("map1", map1);
    QCOMPARE(m_cache->size(), 1);

    m_cache->remove("map1");
    QCOMPARE(m_cache->size(), 0);
}

void TestMapCache::testClear()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto map2 = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    m_cache->add("map1", map1);
    m_cache->add("map2", map2);
    QCOMPARE(m_cache->size(), 2);

    m_cache->clear();
    QCOMPARE(m_cache->size(), 0);
}

void TestMapCache::testContains()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    QVERIFY(!m_cache->contains("map1"));

    m_cache->add("map1", map1);
    QVERIFY(m_cache->contains("map1"));
}

void TestMapCache::testSize()
{
    QCOMPARE(m_cache->size(), 0);

    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    m_cache->add("map1", map1);
    QCOMPARE(m_cache->size(), 1);

    auto map2 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    m_cache->add("map2", map2);
    QCOMPARE(m_cache->size(), 2);
}

void TestMapCache::testMaxSize()
{
    QCOMPARE(m_cache->maxSize(), 10);
}

void TestMapCache::testSetMaxSize()
{
    m_cache->setMaxSize(5);
    QCOMPARE(m_cache->maxSize(), 5);
}

void TestMapCache::testEvictionPolicy()
{
    m_cache->setMaxSize(3);

    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto map2 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto map3 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto map4 = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    m_cache->add("map1", map1);
    QThread::msleep(10);
    m_cache->add("map2", map2);
    QThread::msleep(10);
    m_cache->add("map3", map3);
    QThread::msleep(10);
    m_cache->add("map4", map4);

    QCOMPARE(m_cache->size(), 3);
    QVERIFY(!m_cache->contains("map1"));
}

void TestMapCache::testCacheHit()
{
    auto map1 = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map1->header.frame_id = "test_frame";

    m_cache->add("map1", map1);

    auto retrieved = m_cache->get("map1");
    QVERIFY(retrieved != nullptr);
    QCOMPARE(QString::fromStdString(retrieved->header.frame_id), QString("test_frame"));
}

void TestMapCache::testCacheMiss()
{
    auto retrieved = m_cache->get("nonexistent");
    QVERIFY(retrieved == nullptr);
}
