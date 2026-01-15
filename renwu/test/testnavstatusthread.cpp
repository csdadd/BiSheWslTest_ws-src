#include "testnavstatusthread.h"

void TestNavStatusThread::initTestCase()
{
}

void TestNavStatusThread::cleanupTestCase()
{
}

void TestNavStatusThread::init()
{
    m_thread = new NavStatusThread();
}

void TestNavStatusThread::cleanup()
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

void TestNavStatusThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestNavStatusThread::testInitialize()
{
    QVERIFY(m_thread != nullptr);
}

void TestNavStatusThread::testStart()
{
    QSignalSpy startedSpy(m_thread, &NavStatusThread::threadStarted);

    m_thread->start();
    QThread::msleep(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestNavStatusThread::testStop()
{
    QSignalSpy stoppedSpy(m_thread, &NavStatusThread::threadStopped);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestNavStatusThread::testNavigationStatusSignal()
{
    QSignalSpy statusSpy(m_thread, &NavStatusThread::navigationStatusReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestNavStatusThread::testNavigationFeedbackSignal()
{
    QSignalSpy feedbackSpy(m_thread, &NavStatusThread::navigationFeedbackReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestNavStatusThread::testNavigationPathSignal()
{
    QSignalSpy pathSpy(m_thread, &NavStatusThread::navigationPathReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestNavStatusThread::testConnectionStateSignal()
{
    QSignalSpy connectionSpy(m_thread, &NavStatusThread::connectionStateChanged);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestNavStatusThread::testThreadLifecycle()
{
    QVERIFY(!m_thread->isRunning());

    m_thread->start();
    QThread::msleep(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isRunning());
}
