#ifndef TESTLOGTHREAD_H
#define TESTLOGTHREAD_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include <QFile>
#include "logthread.h"
#include "logstorageengine.h"

class TestLogThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testSetLogFilePath();
    void testGetLogFilePath();
    void testGetStorageEngine();
    void testWriteLog();
    void testWriteLogEntry();
    void testStart();
    void testStop();
    void testThreadLifecycle();
    void testSignalLogFileChanged();
    void testMultipleLogWrites();
    void testLogWithDifferentLevels();
    void testLogWithDifferentSources();

private:
    QString m_testDbPath;
    QString m_testLogPath;
    LogStorageEngine* m_storageEngine;
    LogThread* m_logThread;
};

#endif
