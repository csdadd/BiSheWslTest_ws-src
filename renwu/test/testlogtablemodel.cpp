#include "testlogtablemodel.h"
#include "loglevel.h"

void TestLogTableModel::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestLogTableModel::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestLogTableModel::init()
{
    m_storageEngine = new LogStorageEngine();
    m_storageEngine->initialize(m_testDbPath);

    m_model = new LogTableModel();
    m_model->setStorageEngine(m_storageEngine);
}

void TestLogTableModel::cleanup()
{
    delete m_model;
    m_model = nullptr;
    delete m_storageEngine;
    m_storageEngine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestLogTableModel::testConstructor()
{
    LogTableModel model;
    QVERIFY(model.rowCount() == 0);
    QVERIFY(model.columnCount() > 0);
}

void TestLogTableModel::testRowCount()
{
    QVERIFY(m_model->rowCount() == 0);

    LogEntry entry("Test message", LOG_INFO, QDateTime::currentDateTime());
    m_model->addLogEntry(entry);

    QVERIFY(m_model->rowCount() >= 1);
}

void TestLogTableModel::testColumnCount()
{
    QVERIFY(m_model->columnCount() > 0);
}

void TestLogTableModel::testData()
{
    LogEntry entry("Test message", LOG_INFO, QDateTime::currentDateTime(), "TestSource", "TestCategory");
    m_model->addLogEntry(entry);

    QModelIndex index = m_model->index(0, 0);
    QVERIFY(index.isValid());

    QVariant data = m_model->data(index, Qt::DisplayRole);
    QVERIFY(data.isValid());
}

void TestLogTableModel::testHeaderData()
{
    for (int i = 0; i < m_model->columnCount(); ++i) {
        QVariant header = m_model->headerData(i, Qt::Horizontal, Qt::DisplayRole);
        QVERIFY(header.isValid());
        QVERIFY(!header.toString().isEmpty());
    }
}

void TestLogTableModel::testAddLogEntry()
{
    int initialCount = m_model->rowCount();

    LogEntry entry("Test message", LOG_INFO, QDateTime::currentDateTime());
    m_model->addLogEntry(entry);

    QVERIFY(m_model->rowCount() >= initialCount + 1);
}

void TestLogTableModel::testAddLogEntries()
{
    int initialCount = m_model->rowCount();

    QVector<LogEntry> entries;
    for (int i = 0; i < 5; ++i) {
        entries.append(LogEntry(QString("Message %1").arg(i), LOG_INFO, QDateTime::currentDateTime()));
    }

    m_model->addLogEntries(entries);

    QVERIFY(m_model->rowCount() >= initialCount + 5);
}

void TestLogTableModel::testClearLogs()
{
    for (int i = 0; i < 10; ++i) {
        LogEntry entry(QString("Message %1").arg(i), LOG_INFO, QDateTime::currentDateTime());
        m_model->addLogEntry(entry);
    }

    QVERIFY(m_model->rowCount() > 0);

    m_model->clearLogs();

    QVERIFY(m_model->rowCount() == 0);
}

void TestLogTableModel::testGetLogEntry()
{
    LogEntry entry("Test message", LOG_INFO, QDateTime::currentDateTime(), "TestSource", "TestCategory");
    m_model->addLogEntry(entry);

    LogEntry retrieved = m_model->getLogEntry(0);
    QVERIFY(retrieved.message == "Test message");
    QVERIFY(retrieved.level == LOG_INFO);
    QVERIFY(retrieved.source == "TestSource");
}

void TestLogTableModel::testSetMaxLogs()
{
    m_model->setMaxLogs(100);
    QVERIFY(m_model->getMaxLogs() == 100);
}

void TestLogTableModel::testGetMaxLogs()
{
    QVERIFY(m_model->getMaxLogs() >= 0);
}

void TestLogTableModel::testEnforceMaxLogs()
{
    m_model->setMaxLogs(5);

    for (int i = 0; i < 10; ++i) {
        LogEntry entry(QString("Message %1").arg(i), LOG_INFO, QDateTime::currentDateTime());
        m_model->addLogEntry(entry);
    }

    QVERIFY(m_model->rowCount() <= 5);
}

void TestLogTableModel::testSetStorageEngine()
{
    LogTableModel model;
    QVERIFY(model.getMaxLogs() >= 0);

    model.setStorageEngine(m_storageEngine);
}

void TestLogTableModel::testLoadFromDatabase()
{
    for (int i = 0; i < 5; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LOG_INFO, QDateTime::currentDateTime());
        m_storageEngine->insertLog(entry);
    }

    m_model->loadFromDatabase();

    QVERIFY(m_model->rowCount() >= 5);
}

void TestLogTableModel::testAppendFromDatabase()
{
    for (int i = 0; i < 5; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LOG_INFO, QDateTime::currentDateTime());
        m_storageEngine->insertLog(entry);
    }

    int initialCount = m_model->rowCount();
    m_model->appendFromDatabase();

    QVERIFY(m_model->rowCount() >= initialCount + 5);
}

void TestLogTableModel::testLevelToString()
{
    LogEntry entry1("Debug", LOG_DEBUG, QDateTime::currentDateTime());
    LogEntry entry2("Info", LOG_INFO, QDateTime::currentDateTime());
    LogEntry entry3("Warning", LOG_WARNING, QDateTime::currentDateTime());
    LogEntry entry4("Error", LOG_ERROR, QDateTime::currentDateTime());
    LogEntry entry5("Fatal", LOG_FATAL, QDateTime::currentDateTime());

    m_model->addLogEntry(entry1);
    m_model->addLogEntry(entry2);
    m_model->addLogEntry(entry3);
    m_model->addLogEntry(entry4);
    m_model->addLogEntry(entry5);

    QVERIFY(m_model->rowCount() >= 5);
}

void TestLogTableModel::testConvertStorageEntry()
{
    StorageLogEntry storageEntry("Test message", LOG_INFO, QDateTime::currentDateTime(), "TestSource", "TestCategory");
    m_storageEngine->insertLog(storageEntry);

    m_model->loadFromDatabase();

    QVERIFY(m_model->rowCount() >= 1);
}
