#ifndef TESTHISTORYLOGMODEL_H
#define TESTHISTORYLOGMODEL_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QColor>
#include "historylogmodel.h"
#include "logstorageengine.h"

class TestHistoryLogTableModel : public QObject
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
    void testSetQueryResults();
    void testClear();
    void testGetLogEntry();
    void testGetTotalCount();
    void testSetPaginationInfo();
    void testGetCurrentPage();
    void testGetPageSize();
    void testGetTotalPages();
    void testDataWithEmptySource();
    void testDataWithLevels();
    void testDataTextAlignment();
    void testDataForegroundRole();
    void testInvalidIndex();
    void testMultipleSetQueryResults();
    void testClearWhenEmpty();

private:
    HistoryLogTableModel* m_model;
};

#endif
