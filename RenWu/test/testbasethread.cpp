#include "testbasethread.h"

class MockBaseThread : public BaseThread
{
    Q_OBJECT
public:
    explicit MockBaseThread(QObject* parent = nullptr) : BaseThread(parent) {}

protected:
    void initialize() override {}
    void process() override { QThread::msleep(10); }
    void cleanup() override {}
};

void TestBaseThread::initTestCase()
{
}

void TestBaseThread::cleanupTestCase()
{
}

void TestBaseThread::init()
{
    m_thread = new MockBaseThread();
}

void TestBaseThread::cleanup()
{
    if (m_thread) {
        if (m_thread->isRunning()) {
            m_thread->stopThread();
            m_thread->wait();
        }
        delete m_thread;
        m_thread = nullptr;
    }
}

void TestBaseThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestBaseThread::testStopThread()
{
    QSignalSpy stoppedSpy(m_thread, &BaseThread::threadStopped);

    m_thread->start();
    QThread::msleep(50);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestBaseThread::testIsThreadRunning()
{
    QVERIFY(!m_thread->isThreadRunning());

    m_thread->start();
    QThread::msleep(50);
    QVERIFY(m_thread->isThreadRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isThreadRunning());
}

void TestBaseThread::testThreadStartedSignal()
{
    QSignalSpy startedSpy(m_thread, &BaseThread::threadStarted);

    m_thread->start();
    QThread::msleep(50);

    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestBaseThread::testThreadStoppedSignal()
{
    QSignalSpy stoppedSpy(m_thread, &BaseThread::threadStopped);

    m_thread->start();
    QThread::msleep(50);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(stoppedSpy.count() > 0);
}

void TestBaseThread::testThreadErrorSignal()
{
    QSignalSpy errorSpy(m_thread, &BaseThread::threadError);

    m_thread->start();
    QThread::msleep(50);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(errorSpy.count() == 0);
}

#include "testbasethread.moc"
