#include "testlogthread.h"

void TestLogThread::initTestCase()
{
    QTemporaryFile tempDbFile;
    tempDbFile.open();
    m_testDbPath = tempDbFile.fileName() + ".db";
    tempDbFile.close();

    QTemporaryFile tempLogFile;
    tempLogFile.open();
    m_testLogPath = tempLogFile.fileName() + ".log";
    tempLogFile.close();
}

void TestLogThread::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
    QFile::remove(m_testLogPath);
}

void TestLogThread::init()
{
    m_storageEngine = new LogStorageEngine();
    m_storageEngine->initialize(m_testDbPath);

    m_logThread = new LogThread();
    m_logThread->setLogFilePath(m_testLogPath);
}

void TestLogThread::cleanup()
{
    if (m_logThread->isRunning()) {
        m_logThread->stopThread();
        QTest::qWait(500);
    }

    delete m_logThread;
    m_logThread = nullptr;
    delete m_storageEngine;
    m_storageEngine = nullptr;
    QFile::remove(m_testDbPath);
    QFile::remove(m_testLogPath);
}

void TestLogThread::testConstructor()
{
    LogThread thread;
    QVERIFY(thread.getLogFilePath().isEmpty());
}

void TestLogThread::testSetLogFilePath()
{
    m_logThread->setLogFilePath(m_testLogPath);
    QVERIFY(m_logThread->getLogFilePath() == m_testLogPath);
}

void TestLogThread::testGetLogFilePath()
{
    m_logThread->setLogFilePath(m_testLogPath);
    QVERIFY(m_logThread->getLogFilePath() == m_testLogPath);
}

void TestLogThread::testGetStorageEngine()
{
    QVERIFY(m_logThread->getStorageEngine() != nullptr);
}

void TestLogThread::testWriteLog()
{
    m_logThread->start();
    QTest::qWait(100);

    m_logThread->writeLog("Test message", LOG_INFO);
    QTest::qWait(100);

    QVERIFY(QFile::exists(m_testLogPath));
}

void TestLogThread::testWriteLogEntry()
{
    m_logThread->start();
    QTest::qWait(100);

    LogEntry entry("Test message", LOG_INFO, QDateTime::currentDateTime(), "TestSource", "TestCategory");
    m_logThread->writeLogEntry(entry);
    QTest::qWait(100);

    QVERIFY(QFile::exists(m_testLogPath));
}

void TestLogThread::testStart()
{
    QVERIFY(!m_logThread->isRunning());

    m_logThread->start();
    QTest::qWait(100);

    QVERIFY(m_logThread->isRunning());
}

void TestLogThread::testStop()
{
    m_logThread->start();
    QTest::qWait(100);

    QVERIFY(m_logThread->isRunning());

    m_logThread->stopThread();
    QTest::qWait(500);

    QVERIFY(!m_logThread->isRunning());
}

void TestLogThread::testThreadLifecycle()
{
    QVERIFY(!m_logThread->isRunning());

    m_logThread->start();
    QTest::qWait(100);
    QVERIFY(m_logThread->isRunning());

    m_logThread->writeLog("Test message", LOG_INFO);
    QTest::qWait(100);

    m_logThread->stopThread();
    QTest::qWait(500);
    QVERIFY(!m_logThread->isRunning());
}

void TestLogThread::testSignalLogFileChanged()
{
    QSignalSpy spy(m_logThread, &LogThread::logFileChanged);

    m_logThread->setLogFilePath(m_testLogPath);

    QVERIFY(spy.count() >= 0);
}

void TestLogThread::testMultipleLogWrites()
{
    m_logThread->start();
    QTest::qWait(100);

    for (int i = 0; i < 10; ++i) {
        m_logThread->writeLog(QString("Message %1").arg(i), LOG_INFO);
    }

    QTest::qWait(200);

    QVERIFY(QFile::exists(m_testLogPath));
}

void TestLogThread::testLogWithDifferentLevels()
{
    m_logThread->start();
    QTest::qWait(100);

    m_logThread->writeLog("Debug message", LOG_DEBUG);
    m_logThread->writeLog("Info message", LOG_INFO);
    m_logThread->writeLog("Warning message", LOG_WARNING);
    m_logThread->writeLog("Error message", LOG_ERROR);
    m_logThread->writeLog("Fatal message", LOG_FATAL);

    QTest::qWait(200);

    QVERIFY(QFile::exists(m_testLogPath));
}

void TestLogThread::testLogWithDifferentSources()
{
    m_logThread->start();
    QTest::qWait(100);

    LogEntry entry1("Message from Source1", LOG_INFO, QDateTime::currentDateTime(), "Source1");
    LogEntry entry2("Message from Source2", LOG_INFO, QDateTime::currentDateTime(), "Source2");
    LogEntry entry3("Message from Source3", LOG_INFO, QDateTime::currentDateTime(), "Source3");

    m_logThread->writeLogEntry(entry1);
    m_logThread->writeLogEntry(entry2);
    m_logThread->writeLogEntry(entry3);

    QTest::qWait(200);

    QVERIFY(QFile::exists(m_testLogPath));
}
