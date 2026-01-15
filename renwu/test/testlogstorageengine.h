#ifndef TESTLOGSTORAGEENGINE_H
#define TESTLOGSTORAGEENGINE_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QThread>
#include <QSignalSpy>
#include "logstorageengine.h"

class TestLogStorageEngine : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testInitialize();
    void testInitializeWithCustomPath();
    void testIsInitialized();
    void testInsertLog();
    void testInsertLogs();
    void testQueryLogsByTimeRange();
    void testQueryLogsByLevel();
    void testQueryLogsBySource();
    void testQueryLogsByKeyword();
    void testQueryLogsWithMultipleFilters();
    void testQueryLogsWithLimit();
    void testGetLogCount();
    void testClearLogs();
    void testVacuum();
    void testGetLastError();
    void testSignalLogInserted();
    void testSignalLogQueryCompleted();
    void testSignalErrorOccurred();
    void testThreadSafety();

private:
    QString m_testDbPath;
    LogStorageEngine* m_engine;
};

#endif
