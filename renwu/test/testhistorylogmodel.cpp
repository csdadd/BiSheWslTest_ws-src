#include "testhistorylogmodel.h"
#include "loglevel.h"

void TestHistoryLogTableModel::initTestCase()
{
}

void TestHistoryLogTableModel::cleanupTestCase()
{
}

void TestHistoryLogTableModel::init()
{
    m_model = new HistoryLogTableModel();
}

void TestHistoryLogTableModel::cleanup()
{
    delete m_model;
    m_model = nullptr;
}

void TestHistoryLogTableModel::testConstructor()
{
    HistoryLogTableModel model;
    QVERIFY(model.rowCount() == 0);
    QVERIFY(model.columnCount() == 4);
    QVERIFY(model.getTotalCount() == 0);
    QVERIFY(model.getCurrentPage() == 1);
    QVERIFY(model.getPageSize() == 100);
}

void TestHistoryLogTableModel::testRowCount()
{
    QVERIFY(m_model->rowCount() == 0);

    QVector<StorageLogEntry> entries;
    entries.append(StorageLogEntry("Message 1", LogLevel::INFO, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Message 2", LogLevel::WARNING, QDateTime::currentDateTime()));

    m_model->setQueryResults(entries);

    QVERIFY(m_model->rowCount() == 2);
}

void TestHistoryLogTableModel::testColumnCount()
{
    QVERIFY(m_model->columnCount() == 4);
}

void TestHistoryLogTableModel::testData()
{
    QDateTime testTime = QDateTime::fromString("2026-02-07 12:00:00", "yyyy-MM-dd hh:mm:ss");
    StorageLogEntry entry("Test message", LogLevel::INFO, testTime, "TestSource");
    QVector<StorageLogEntry> entries;
    entries.append(entry);

    m_model->setQueryResults(entries);

    QModelIndex index0 = m_model->index(0, 0);
    QVERIFY(index0.isValid());
    QVariant data0 = m_model->data(index0, Qt::DisplayRole);
    QVERIFY(data0.isValid());
    QVERIFY(data0.toString().contains("2026-02-07"));

    QModelIndex index1 = m_model->index(0, 1);
    QVariant data1 = m_model->data(index1, Qt::DisplayRole);
    QVERIFY(data1.toString() == "INFO");

    QModelIndex index2 = m_model->index(0, 2);
    QVariant data2 = m_model->data(index2, Qt::DisplayRole);
    QVERIFY(data2.toString() == "TestSource");

    QModelIndex index3 = m_model->index(0, 3);
    QVariant data3 = m_model->data(index3, Qt::DisplayRole);
    QVERIFY(data3.toString() == "Test message");
}

void TestHistoryLogTableModel::testHeaderData()
{
    for (int i = 0; i < m_model->columnCount(); ++i) {
        QVariant header = m_model->headerData(i, Qt::Horizontal, Qt::DisplayRole);
        QVERIFY(header.isValid());
        QVERIFY(!header.toString().isEmpty());
    }

    QVariant header0 = m_model->headerData(0, Qt::Horizontal, Qt::DisplayRole);
    QVERIFY(header0.toString() == "时间");

    QVariant header1 = m_model->headerData(1, Qt::Horizontal, Qt::DisplayRole);
    QVERIFY(header1.toString() == "级别");

    QVariant header2 = m_model->headerData(2, Qt::Horizontal, Qt::DisplayRole);
    QVERIFY(header2.toString() == "来源");

    QVariant header3 = m_model->headerData(3, Qt::Horizontal, Qt::DisplayRole);
    QVERIFY(header3.toString() == "消息");
}

void TestHistoryLogTableModel::testSetQueryResults()
{
    QVector<StorageLogEntry> entries;
    for (int i = 0; i < 10; ++i) {
        entries.append(StorageLogEntry(QString("Message %1").arg(i),
                                      LogLevel::INFO,
                                      QDateTime::currentDateTime()));
    }

    m_model->setQueryResults(entries);

    QVERIFY(m_model->rowCount() == 10);
}

void TestHistoryLogTableModel::testClear()
{
    QVector<StorageLogEntry> entries;
    for (int i = 0; i < 10; ++i) {
        entries.append(StorageLogEntry(QString("Message %1").arg(i),
                                      LogLevel::INFO,
                                      QDateTime::currentDateTime()));
    }

    m_model->setQueryResults(entries);
    m_model->setPaginationInfo(100, 2, 50);

    QVERIFY(m_model->rowCount() == 10);

    m_model->clear();

    QVERIFY(m_model->rowCount() == 0);
    QVERIFY(m_model->getTotalCount() == 0);
    QVERIFY(m_model->getCurrentPage() == 1);
}

void TestHistoryLogTableModel::testGetLogEntry()
{
    StorageLogEntry entry("Test message", LogLevel::ERROR, QDateTime::currentDateTime(), "TestSource");
    QVector<StorageLogEntry> entries;
    entries.append(entry);

    m_model->setQueryResults(entries);

    StorageLogEntry retrieved = m_model->getLogEntry(0);
    QVERIFY(retrieved.message == "Test message");
    QVERIFY(retrieved.level == LogLevel::ERROR);
    QVERIFY(retrieved.source == "TestSource");

    StorageLogEntry invalid = m_model->getLogEntry(100);
    QVERIFY(invalid.message.isEmpty());
}

void TestHistoryLogTableModel::testGetTotalCount()
{
    QVERIFY(m_model->getTotalCount() == 0);

    m_model->setPaginationInfo(500, 1, 100);

    QVERIFY(m_model->getTotalCount() == 500);
}

void TestHistoryLogTableModel::testSetPaginationInfo()
{
    m_model->setPaginationInfo(250, 3, 50);

    QVERIFY(m_model->getTotalCount() == 250);
    QVERIFY(m_model->getCurrentPage() == 3);
    QVERIFY(m_model->getPageSize() == 50);
}

void TestHistoryLogTableModel::testGetCurrentPage()
{
    QVERIFY(m_model->getCurrentPage() == 1);

    m_model->setPaginationInfo(100, 5, 20);

    QVERIFY(m_model->getCurrentPage() == 5);
}

void TestHistoryLogTableModel::testGetPageSize()
{
    QVERIFY(m_model->getPageSize() == 100);

    m_model->setPaginationInfo(100, 1, 200);

    QVERIFY(m_model->getPageSize() == 200);
}

void TestHistoryLogTableModel::testGetTotalPages()
{
    m_model->setPaginationInfo(0, 1, 100);
    QVERIFY(m_model->getTotalPages() == 0);

    m_model->setPaginationInfo(100, 1, 100);
    QVERIFY(m_model->getTotalPages() == 1);

    m_model->setPaginationInfo(150, 1, 100);
    QVERIFY(m_model->getTotalPages() == 2);

    m_model->setPaginationInfo(250, 1, 50);
    QVERIFY(m_model->getTotalPages() == 5);

    m_model->setPaginationInfo(999, 1, 100);
    QVERIFY(m_model->getTotalPages() == 10);
}

void TestHistoryLogTableModel::testDataWithEmptySource()
{
    StorageLogEntry entry("Test message", LogLevel::INFO, QDateTime::currentDateTime(), "");
    QVector<StorageLogEntry> entries;
    entries.append(entry);

    m_model->setQueryResults(entries);

    QModelIndex index = m_model->index(0, 2);
    QVariant data = m_model->data(index, Qt::DisplayRole);

    QVERIFY(data.toString() == "系统");
}

void TestHistoryLogTableModel::testDataWithLevels()
{
    QVector<StorageLogEntry> entries;
    entries.append(StorageLogEntry("Debug message", LogLevel::DEBUG, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Info message", LogLevel::INFO, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Warning message", LogLevel::WARNING, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Error message", LogLevel::ERROR, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Fatal message", LogLevel::FATAL, QDateTime::currentDateTime()));

    m_model->setQueryResults(entries);

    QVERIFY(m_model->data(m_model->index(0, 1), Qt::DisplayRole).toString() == "DEBUG");
    QVERIFY(m_model->data(m_model->index(1, 1), Qt::DisplayRole).toString() == "INFO");
    QVERIFY(m_model->data(m_model->index(2, 1), Qt::DisplayRole).toString() == "WARN");
    QVERIFY(m_model->data(m_model->index(3, 1), Qt::DisplayRole).toString() == "ERROR");
    QVERIFY(m_model->data(m_model->index(4, 1), Qt::DisplayRole).toString() == "FATAL");
}

void TestHistoryLogTableModel::testDataTextAlignment()
{
    StorageLogEntry entry("Test", LogLevel::INFO, QDateTime::currentDateTime());
    QVector<StorageLogEntry> entries;
    entries.append(entry);

    m_model->setQueryResults(entries);

    QVariant align0 = m_model->data(m_model->index(0, 0), Qt::TextAlignmentRole);
    QVERIFY(align0.isValid());
    QVERIFY(align0.toInt() == (Qt::AlignLeft | Qt::AlignVCenter));

    QVariant align1 = m_model->data(m_model->index(0, 1), Qt::TextAlignmentRole);
    QVERIFY(align1.isValid());
    QVERIFY(align1.toInt() == Qt::AlignCenter);
}

void TestHistoryLogTableModel::testDataForegroundRole()
{
    QVector<StorageLogEntry> entries;
    entries.append(StorageLogEntry("Debug", LogLevel::DEBUG, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Info", LogLevel::INFO, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Warning", LogLevel::WARNING, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Error", LogLevel::ERROR, QDateTime::currentDateTime()));
    entries.append(StorageLogEntry("Fatal", LogLevel::FATAL, QDateTime::currentDateTime()));

    m_model->setQueryResults(entries);

    QColor debugColor = m_model->data(m_model->index(0, 3), Qt::ForegroundRole).value<QColor>();
    QVERIFY(debugColor == QColor(128, 128, 128));

    QColor infoColor = m_model->data(m_model->index(1, 3), Qt::ForegroundRole).value<QColor>();
    QVERIFY(infoColor == QColor(0, 0, 0));

    QColor warningColor = m_model->data(m_model->index(2, 3), Qt::ForegroundRole).value<QColor>();
    QVERIFY(warningColor == QColor(255, 165, 0));

    QColor errorColor = m_model->data(m_model->index(3, 3), Qt::ForegroundRole).value<QColor>();
    QVERIFY(errorColor == QColor(255, 0, 0));

    QColor fatalColor = m_model->data(m_model->index(4, 3), Qt::ForegroundRole).value<QColor>();
    QVERIFY(fatalColor == QColor(139, 0, 0));
}

void TestHistoryLogTableModel::testInvalidIndex()
{
    QVariant data = m_model->data(QModelIndex(), Qt::DisplayRole);
    QVERIFY(!data.isValid());

    QVariant data2 = m_model->data(m_model->index(0, 10), Qt::DisplayRole);
    QVERIFY(!data2.isValid());
}

void TestHistoryLogTableModel::testMultipleSetQueryResults()
{
    QVector<StorageLogEntry> entries1;
    for (int i = 0; i < 5; ++i) {
        entries1.append(StorageLogEntry(QString("Msg1_%1").arg(i), LogLevel::INFO, QDateTime::currentDateTime()));
    }

    m_model->setQueryResults(entries1);
    QVERIFY(m_model->rowCount() == 5);

    QVector<StorageLogEntry> entries2;
    for (int i = 0; i < 3; ++i) {
        entries2.append(StorageLogEntry(QString("Msg2_%1").arg(i), LogLevel::WARNING, QDateTime::currentDateTime()));
    }

    m_model->setQueryResults(entries2);
    QVERIFY(m_model->rowCount() == 3);

    QVERIFY(m_model->data(m_model->index(0, 3), Qt::DisplayRole).toString() == "Msg2_0");
}

void TestHistoryLogTableModel::testClearWhenEmpty()
{
    QVERIFY(m_model->rowCount() == 0);

    m_model->clear();

    QVERIFY(m_model->rowCount() == 0);
}
