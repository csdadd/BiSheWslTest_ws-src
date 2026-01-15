#ifndef TESTMAPLOADER_H
#define TESTMAPLOADER_H

#include <QtTest>
#include "maploader.h"
#include "mapcache.h"

class TestMapLoader : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testParseYaml();
    void testLoadPgm();
    void testLoadFromFile();
    void testMapCache();
    void testMapCacheEviction();

private:
    QString m_testDataDir;
};

#endif
