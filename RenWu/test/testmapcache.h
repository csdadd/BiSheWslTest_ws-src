#ifndef TESTMAPCACHE_H
#define TESTMAPCACHE_H

#include <QtTest/QtTest>
#include <QThread>
#include "mapcache.h"

class TestMapCache : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testAdd();
    void testGet();
    void testRemove();
    void testClear();
    void testContains();
    void testSize();
    void testMaxSize();
    void testSetMaxSize();
    void testEvictionPolicy();
    void testCacheHit();
    void testCacheMiss();

private:
    MapCache* m_cache;
};

#endif
