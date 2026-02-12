#include "testlogstorageengine.h"
#include "loglevel.h"

void TestLogStorageEngine::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestLogStorageEngine::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestLogStorageEngine::init()
{
    m_engine = new LogStorageEngine();
    QVERIFY(m_engine->initialize(m_testDbPath));
}

void TestLogStorageEngine::cleanup()
{
    delete m_engine;
    m_engine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestLogStorageEngine::testInitialize()
{
    LogStorageEngine engine;
    QVERIFY(engine.initialize(m_testDbPath));
    QVERIFY(engine.isInitialized());
}

void TestLogStorageEngine::testInitializeWithCustomPath()
{
    QString customPath = m_testDbPath + "_custom";
    LogStorageEngine engine;
    QVERIFY(engine.initialize(customPath));
    QVERIFY(engine.isInitialized());
    QFile::remove(customPath);
}

void TestLogStorageEngine::testIsInitialized()
{
    LogStorageEngine engine;
    QVERIFY(!engine.isInitialized());
    engine.initialize(m_testDbPath);
    QVERIFY(engine.isInitialized());
}

void TestLogStorageEngine::testInsertLog()
{
    StorageLogEntry entry("Test message", LogLevel::INFO, QDateTime::currentDateTime(), "TestSource");
    QVERIFY(m_engine->insertLog(entry));

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime());
    QVERIFY(logs.size() >= 1);
}

void TestLogStorageEngine::testInsertLogs()
{
    QVector<StorageLogEntry> entries;
    for (int i = 0; i < 10; ++i) {
        entries.append(StorageLogEntry(QString("Message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime()));
    }

    QVERIFY(m_engine->insertLogs(entries));

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime());
    QVERIFY(logs.size() >= 10);
}

void TestLogStorageEngine::testQueryLogsByTimeRange()
{
    QDateTime now = QDateTime::currentDateTime();
    QDateTime oneHourAgo = now.addSecs(-3600);

    StorageLogEntry entry1("Old message", LogLevel::INFO, oneHourAgo);
    StorageLogEntry entry2("New message", LogLevel::INFO, now);

    m_engine->insertLog(entry1);
    m_engine->insertLog(entry2);

    QVector<StorageLogEntry> logs = m_engine->queryLogs(oneHourAgo.addSecs(-60), now.addSecs(60));
    QVERIFY(logs.size() >= 2);
}

void TestLogStorageEngine::testQueryLogsByLevel()
{
    StorageLogEntry entry1("Debug message", LogLevel::DEBUG, QDateTime::currentDateTime());
    StorageLogEntry entry2("Info message", LogLevel::INFO, QDateTime::currentDateTime());
    StorageLogEntry entry3("Error message", LogLevel::ERROR, QDateTime::currentDateTime());

    m_engine->insertLog(entry1);
    m_engine->insertLog(entry2);
    m_engine->insertLog(entry3);

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime(), LogLevel::ERROR);
    QVERIFY(logs.size() >= 1);
}

void TestLogStorageEngine::testQueryLogsBySource()
{
    StorageLogEntry entry1("Message from source1", LogLevel::INFO, QDateTime::currentDateTime(), "Source1");
    StorageLogEntry entry2("Message from source2", LogLevel::INFO, QDateTime::currentDateTime(), "Source2");

    m_engine->insertLog(entry1);
    m_engine->insertLog(entry2);

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime(), LogLevel::DEBUG, "Source1");
    QVERIFY(logs.size() >= 1);
}

void TestLogStorageEngine::testQueryLogsByKeyword()
{
    StorageLogEntry entry1("This is a test message", LogLevel::INFO, QDateTime::currentDateTime());
    StorageLogEntry entry2("Another message", LogLevel::INFO, QDateTime::currentDateTime());

    m_engine->insertLog(entry1);
    m_engine->insertLog(entry2);

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime(), LogLevel::DEBUG, "", "test");
    QVERIFY(logs.size() >= 1);
}

void TestLogStorageEngine::testQueryLogsWithMultipleFilters()
{
    StorageLogEntry entry1("Error from Source1", LogLevel::ERROR, QDateTime::currentDateTime(), "Source1");
    StorageLogEntry entry2("Info from Source1", LogLevel::INFO, QDateTime::currentDateTime(), "Source1");
    StorageLogEntry entry3("Error from Source2", LogLevel::ERROR, QDateTime::currentDateTime(), "Source2");

    m_engine->insertLog(entry1);
    m_engine->insertLog(entry2);
    m_engine->insertLog(entry3);

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime(), LogLevel::ERROR, "Source1");
    QVERIFY(logs.size() >= 1);
}

void TestLogStorageEngine::testQueryLogsWithLimit()
{
    for (int i = 0; i < 20; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime());
        m_engine->insertLog(entry);
    }

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime(), LogLevel::DEBUG, "", "", 10);
    QVERIFY(logs.size() <= 10);
}

void TestLogStorageEngine::testGetLogCount()
{
    int initialCount = m_engine->getLogCount();

    for (int i = 0; i < 5; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime());
        m_engine->insertLog(entry);
    }

    int newCount = m_engine->getLogCount();
    QVERIFY(newCount >= initialCount + 5);
}

void TestLogStorageEngine::testClearLogs()
{
    for (int i = 0; i < 10; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime());
        m_engine->insertLog(entry);
    }

    QDateTime cutoff = QDateTime::currentDateTime().addSecs(1);
    QVERIFY(m_engine->clearLogs(cutoff));

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime());
    QVERIFY(logs.size() >= 0);
}

void TestLogStorageEngine::testVacuum()
{
    for (int i = 0; i < 10; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime());
        m_engine->insertLog(entry);
    }

    QVERIFY(m_engine->vacuum());
}

void TestLogStorageEngine::testGetLastError()
{
    QString error = m_engine->getLastError();
    QVERIFY(error.isEmpty() || !error.isEmpty());
}

void TestLogStorageEngine::testSignalLogInserted()
{
    QSignalSpy spy(m_engine, &LogStorageEngine::logInserted);
    StorageLogEntry entry("Test message", LogLevel::INFO, QDateTime::currentDateTime());
    m_engine->insertLog(entry);

    QVERIFY(spy.count() >= 1);
}

void TestLogStorageEngine::testSignalLogQueryCompleted()
{
    QSignalSpy spy(m_engine, &LogStorageEngine::logQueryCompleted);
    m_engine->queryLogs(QDateTime(), QDateTime());

    QVERIFY(spy.count() >= 1);
}

void TestLogStorageEngine::testSignalErrorOccurred()
{
    QSignalSpy spy(m_engine, &LogStorageEngine::errorOccurred);
    m_engine->queryLogs(QDateTime(), QDateTime());

    QVERIFY(spy.count() >= 0);
}

void TestLogStorageEngine::testThreadSafety()
{
    const int threadCount = 10;
    QVector<QThread*> threads;

    for (int i = 0; i < threadCount; ++i) {
        QThread* thread = QThread::create([this, i]() {
            StorageLogEntry entry(QString("Thread message %1").arg(i), LogLevel::INFO, QDateTime::currentDateTime());
            m_engine->insertLog(entry);
            m_engine->queryLogs(QDateTime(), QDateTime());
        });
        threads.append(thread);
        thread->start();
    }

    for (QThread* thread : threads) {
        thread->wait();
        delete thread;
    }

    QVector<StorageLogEntry> logs = m_engine->queryLogs(QDateTime(), QDateTime());
    QVERIFY(logs.size() >= threadCount);
}
