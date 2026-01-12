#ifndef TESTLOGTABLEMODEL_H
#define TESTLOGTABLEMODEL_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include "logtablemodel.h"
#include "logstorageengine.h"

class TestLogTableModel : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testRowCount();
    void testColumnCount();
    void testData();
    void testHeaderData();
    void testAddLogEntry();
    void testAddLogEntries();
    void testClearLogs();
    void testGetLogEntry();
    void testSetMaxLogs();
    void testGetMaxLogs();
    void testEnforceMaxLogs();
    void testSetStorageEngine();
    void testLoadFromDatabase();
    void testAppendFromDatabase();
    void testLevelToString();
    void testConvertStorageEntry();

private:
    QString m_testDbPath;
    LogStorageEngine* m_storageEngine;
    LogTableModel* m_model;
};

#endif
