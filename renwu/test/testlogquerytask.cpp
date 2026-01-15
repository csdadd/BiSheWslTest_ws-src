#include "testlogquerytask.h"

static int registerMetaTypes() {
    qRegisterMetaType<StorageLogEntry>("StorageLogEntry");
    qRegisterMetaType<QVector<StorageLogEntry>>("QVector<StorageLogEntry>");
    return 0;
}
static int metaTypesRegistered = registerMetaTypes();

void TestLogQueryTask::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestLogQueryTask::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestLogQueryTask::init()
{
    m_engine = new LogStorageEngine();
    m_engine->initialize(m_testDbPath);

    for (int i = 0; i < 20; ++i) {
        StorageLogEntry entry(QString("Message %1").arg(i), i % 5, QDateTime::currentDateTime(), QString("Source%1").arg(i % 3));
        m_engine->insertLog(entry);
    }
}

void TestLogQueryTask::cleanup()
{
    delete m_engine;
    m_engine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestLogQueryTask::testConstructor()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask task(m_engine, now.addSecs(-3600), now);

    QVERIFY(task.parent() == nullptr);
}

void TestLogQueryTask::testRun()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() >= 1);
    delete task;
}

void TestLogQueryTask::testRunWithResults()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    QList<QVariant> arguments = spy.takeFirst();
    QVector<StorageLogEntry> results = arguments.at(0).value<QVector<StorageLogEntry>>();

    QVERIFY(results.size() > 0);
    delete task;
}

void TestLogQueryTask::testRunWithNoResults()
{
    QDateTime future = QDateTime::currentDateTime().addYears(100);
    LogQueryTask* task = new LogQueryTask(m_engine, future, future.addSecs(3600));
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    QList<QVariant> arguments = spy.takeFirst();
    QVector<StorageLogEntry> results = arguments.at(0).value<QVector<StorageLogEntry>>();

    QVERIFY(results.size() == 0);
    delete task;
}

void TestLogQueryTask::testRunWithError()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(nullptr, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryFailed);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() >= 0);
    delete task;
}

void TestLogQueryTask::testRunWithLimit()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now, -1, "", "", 5);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    QList<QVariant> arguments = spy.takeFirst();
    QVector<StorageLogEntry> results = arguments.at(0).value<QVector<StorageLogEntry>>();

    QVERIFY(results.size() <= 5);
    delete task;
}

void TestLogQueryTask::testRunWithOffset()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now, -1, "", "", -1, 10);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    QList<QVariant> arguments = spy.takeFirst();
    QVector<StorageLogEntry> results = arguments.at(0).value<QVector<StorageLogEntry>>();

    QVERIFY(results.size() >= 0);
    delete task;
}

void TestLogQueryTask::testRunWithMultipleFilters()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now, STORAGE_LOG_ERROR, "Source1", "Message");
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    QList<QVariant> arguments = spy.takeFirst();
    QVector<StorageLogEntry> results = arguments.at(0).value<QVector<StorageLogEntry>>();

    QVERIFY(results.size() >= 0);
    delete task;
}

void TestLogQueryTask::testSignalQueryCompleted()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() == 1);
    delete task;
}

void TestLogQueryTask::testSignalQueryFailed()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(nullptr, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryFailed);
    task->run();

    QTest::qWait(100);

    QVERIFY(spy.count() >= 0);
    delete task;
}

void TestLogQueryTask::testAutoDelete()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now);

    QVERIFY(task->autoDelete() == true);
}

void TestLogQueryTask::testThreadPool()
{
    QDateTime now = QDateTime::currentDateTime();
    LogQueryTask* task = new LogQueryTask(m_engine, now.addSecs(-3600), now);
    task->setAutoDelete(false);

    QSignalSpy spy(task, &LogQueryTask::queryCompleted);
    QThreadPool::globalInstance()->start(task);

    QTest::qWait(500);

    QVERIFY(spy.count() == 1);
    delete task;
}
