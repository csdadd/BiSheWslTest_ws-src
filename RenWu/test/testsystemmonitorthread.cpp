#include "testsystemmonitorthread.h"

void TestSystemMonitorThread::initTestCase()
{
}

void TestSystemMonitorThread::cleanupTestCase()
{
}

void TestSystemMonitorThread::init()
{
    m_thread = new SystemMonitorThread();
}

void TestSystemMonitorThread::cleanup()
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

void TestSystemMonitorThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestSystemMonitorThread::testInitialize()
{
    QVERIFY(m_thread != nullptr);
}

void TestSystemMonitorThread::testStart()
{
    QSignalSpy startedSpy(m_thread, &SystemMonitorThread::threadStarted);

    m_thread->start();
    QThread::msleep(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testStop()
{
    QSignalSpy stoppedSpy(m_thread, &SystemMonitorThread::threadStopped);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestSystemMonitorThread::testLogMessageSignal()
{
    QSignalSpy logSpy(m_thread, &SystemMonitorThread::logMessageReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testCollisionDetectedSignal()
{
    QSignalSpy collisionSpy(m_thread, &SystemMonitorThread::collisionDetected);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testAnomalyDetectedSignal()
{
    QSignalSpy anomalySpy(m_thread, &SystemMonitorThread::anomalyDetected);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testBehaviorTreeLogSignal()
{
    QSignalSpy btLogSpy(m_thread, &SystemMonitorThread::behaviorTreeLogReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testConnectionStateSignal()
{
    QSignalSpy connectionSpy(m_thread, &SystemMonitorThread::connectionStateChanged);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestSystemMonitorThread::testOnDiagnosticsReceived()
{
    QSignalSpy logSpy(m_thread, &SystemMonitorThread::logMessageReceived);

    emit m_thread->onDiagnosticsReceived("OK", 0, "Test message");
}

void TestSystemMonitorThread::testThreadLifecycle()
{
    QVERIFY(!m_thread->isRunning());

    m_thread->start();
    QThread::msleep(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isRunning());
}
