#ifndef TESTLOGFILTERPROXYMODEL_H
#define TESTLOGFILTERPROXYMODEL_H

#include <QtTest/QtTest>
#include <QSet>
#include <QDateTime>
#include "logfilterproxymodel.h"
#include "logtablemodel.h"

class TestLogFilterProxyModel : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testSetLogLevelFilter();
    void testLogLevelFilter();
    void testSetKeywordFilter();
    void testKeywordFilter();
    void testSetSourceFilter();
    void testSourceFilter();
    void testSetTimeRangeFilter();
    void testClearTimeRangeFilter();
    void testStartTime();
    void testEndTime();
    void testSetRegExpFilter();
    void testSetUseRegExp();
    void testRegExpFilter();
    void testUseRegExp();
    void testClearAllFilters();
    void testFilterAcceptsRow();
    void testMultipleFilters();

private:
    LogTableModel* m_sourceModel;
    LogFilterProxyModel* m_proxyModel;
};

#endif
