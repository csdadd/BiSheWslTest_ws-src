#ifndef TESTLOGQUERYTASK_H
#define TESTLOGQUERYTASK_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include <QThreadPool>
#include "logquerytask.h"
#include "logstorageengine.h"

class TestLogQueryTask : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testRun();
    void testRunWithResults();
    void testRunWithNoResults();
    void testRunWithError();
    void testRunWithLimit();
    void testRunWithOffset();
    void testRunWithMultipleFilters();
    void testSignalQueryCompleted();
    void testSignalQueryFailed();
    void testAutoDelete();
    void testThreadPool();

private:
    QString m_testDbPath;
    LogStorageEngine* m_engine;
};

#endif
