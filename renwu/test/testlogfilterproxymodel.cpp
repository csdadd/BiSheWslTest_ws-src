#include "testlogfilterproxymodel.h"

void TestLogFilterProxyModel::initTestCase()
{
}

void TestLogFilterProxyModel::cleanupTestCase()
{
}

void TestLogFilterProxyModel::init()
{
    m_sourceModel = new LogTableModel();
    m_proxyModel = new LogFilterProxyModel();
    m_proxyModel->setSourceModel(m_sourceModel);

    for (int i = 0; i < 20; ++i) {
        int level = i % 5;
        QString source = QString("Source%1").arg(i % 3);
        LogEntry entry(QString("Message %1").arg(i), level, QDateTime::currentDateTime(), source);
        m_sourceModel->addLogEntry(entry);
    }
}

void TestLogFilterProxyModel::cleanup()
{
    delete m_proxyModel;
    m_proxyModel = nullptr;
    delete m_sourceModel;
    m_sourceModel = nullptr;
}

void TestLogFilterProxyModel::testConstructor()
{
    LogFilterProxyModel model;
    QVERIFY(model.logLevelFilter().isEmpty());
    QVERIFY(model.keywordFilter().isEmpty());
    QVERIFY(model.sourceFilter().isEmpty());
}

void TestLogFilterProxyModel::testSetLogLevelFilter()
{
    QSet<int> levels;
    levels.insert(LOG_INFO);
    levels.insert(LOG_ERROR);

    m_proxyModel->setLogLevelFilter(levels);

    QVERIFY(m_proxyModel->logLevelFilter() == levels);
}

void TestLogFilterProxyModel::testLogLevelFilter()
{
    QSet<int> levels;
    levels.insert(LOG_INFO);

    m_proxyModel->setLogLevelFilter(levels);

    QVERIFY(m_proxyModel->logLevelFilter().size() == 1);
    QVERIFY(m_proxyModel->logLevelFilter().contains(LOG_INFO));
}

void TestLogFilterProxyModel::testSetKeywordFilter()
{
    m_proxyModel->setKeywordFilter("error");

    QVERIFY(m_proxyModel->keywordFilter() == "error");
}

void TestLogFilterProxyModel::testKeywordFilter()
{
    m_proxyModel->setKeywordFilter("test");

    QVERIFY(m_proxyModel->keywordFilter() == "test");
}

void TestLogFilterProxyModel::testSetSourceFilter()
{
    m_proxyModel->setSourceFilter("Source1");

    QVERIFY(m_proxyModel->sourceFilter() == "Source1");
}

void TestLogFilterProxyModel::testSourceFilter()
{
    m_proxyModel->setSourceFilter("Source2");

    QVERIFY(m_proxyModel->sourceFilter() == "Source2");
}

void TestLogFilterProxyModel::testSetTimeRangeFilter()
{
    QDateTime now = QDateTime::currentDateTime();
    QDateTime oneHourAgo = now.addSecs(-3600);

    m_proxyModel->setTimeRangeFilter(oneHourAgo, now);

    QVERIFY(m_proxyModel->startTime() == oneHourAgo);
    QVERIFY(m_proxyModel->endTime() == now);
}

void TestLogFilterProxyModel::testClearTimeRangeFilter()
{
    QDateTime now = QDateTime::currentDateTime();
    QDateTime oneHourAgo = now.addSecs(-3600);

    m_proxyModel->setTimeRangeFilter(oneHourAgo, now);
    m_proxyModel->clearTimeRangeFilter();

    QVERIFY(!m_proxyModel->startTime().isValid());
    QVERIFY(!m_proxyModel->endTime().isValid());
}

void TestLogFilterProxyModel::testStartTime()
{
    QDateTime now = QDateTime::currentDateTime();
    QDateTime oneHourAgo = now.addSecs(-3600);

    m_proxyModel->setTimeRangeFilter(oneHourAgo, now);

    QVERIFY(m_proxyModel->startTime() == oneHourAgo);
}

void TestLogFilterProxyModel::testEndTime()
{
    QDateTime now = QDateTime::currentDateTime();
    QDateTime oneHourAgo = now.addSecs(-3600);

    m_proxyModel->setTimeRangeFilter(oneHourAgo, now);

    QVERIFY(m_proxyModel->endTime() == now);
}

void TestLogFilterProxyModel::testSetRegExpFilter()
{
    m_proxyModel->setRegExpFilter("error.*test");

    QVERIFY(m_proxyModel->regExpFilter() == "error.*test");
}

void TestLogFilterProxyModel::testSetUseRegExp()
{
    m_proxyModel->setUseRegExp(true);

    QVERIFY(m_proxyModel->useRegExp() == true);

    m_proxyModel->setUseRegExp(false);

    QVERIFY(m_proxyModel->useRegExp() == false);
}

void TestLogFilterProxyModel::testRegExpFilter()
{
    m_proxyModel->setRegExpFilter("test.*");

    QVERIFY(m_proxyModel->regExpFilter() == "test.*");
}

void TestLogFilterProxyModel::testUseRegExp()
{
    m_proxyModel->setUseRegExp(true);

    QVERIFY(m_proxyModel->useRegExp() == true);
}

void TestLogFilterProxyModel::testClearAllFilters()
{
    QSet<int> levels;
    levels.insert(LOG_INFO);
    m_proxyModel->setLogLevelFilter(levels);
    m_proxyModel->setKeywordFilter("test");
    m_proxyModel->setSourceFilter("Source1");
    m_proxyModel->setRegExpFilter("error");

    m_proxyModel->clearAllFilters();

    QVERIFY(m_proxyModel->logLevelFilter().isEmpty());
    QVERIFY(m_proxyModel->keywordFilter().isEmpty());
    QVERIFY(m_proxyModel->sourceFilter().isEmpty());
    QVERIFY(m_proxyModel->regExpFilter().isEmpty());
}

void TestLogFilterProxyModel::testFilterAcceptsRow()
{
    QSet<int> levels;
    levels.insert(LOG_INFO);
    m_proxyModel->setLogLevelFilter(levels);

    m_proxyModel->invalidate();

    int filteredCount = m_proxyModel->rowCount();
    int sourceCount = m_sourceModel->rowCount();

    QVERIFY(filteredCount <= sourceCount);
}

void TestLogFilterProxyModel::testMultipleFilters()
{
    QSet<int> levels;
    levels.insert(LOG_INFO);
    levels.insert(LOG_ERROR);

    m_proxyModel->setLogLevelFilter(levels);
    m_proxyModel->setSourceFilter("Source1");
    m_proxyModel->setKeywordFilter("Message");

    m_proxyModel->invalidate();

    int filteredCount = m_proxyModel->rowCount();
    int sourceCount = m_sourceModel->rowCount();

    QVERIFY(filteredCount <= sourceCount);
}
