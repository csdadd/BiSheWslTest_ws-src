#include "testinfothread.h"

void TestInfoThread::initTestCase()
{
}

void TestInfoThread::cleanupTestCase()
{
}

void TestInfoThread::init()
{
    m_thread = new InfoThread();
}

void TestInfoThread::cleanup()
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

void TestInfoThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestInfoThread::testInitialize()
{
    QVERIFY(m_thread != nullptr);
}

void TestInfoThread::testStart()
{
    QSignalSpy startedSpy(m_thread, &InfoThread::threadStarted);

    m_thread->start();
    QThread::msleep(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testStop()
{
    QSignalSpy stoppedSpy(m_thread, &InfoThread::threadStopped);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestInfoThread::testBatteryStatusSignal()
{
    QSignalSpy batterySpy(m_thread, &InfoThread::batteryStatusReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testPositionSignal()
{
    QSignalSpy positionSpy(m_thread, &InfoThread::positionReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testOdometrySignal()
{
    QSignalSpy odometrySpy(m_thread, &InfoThread::odometryReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testSystemStatusSignal()
{
    QSignalSpy statusSpy(m_thread, &InfoThread::systemStatusReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testSystemTimeSignal()
{
    QSignalSpy timeSpy(m_thread, &InfoThread::systemTimeReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testNavigationStatusSignal()
{
    QSignalSpy navStatusSpy(m_thread, &InfoThread::navigationStatusReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testNavigationFeedbackSignal()
{
    QSignalSpy navFeedbackSpy(m_thread, &InfoThread::navigationFeedbackReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testNavigationPathSignal()
{
    QSignalSpy navPathSpy(m_thread, &InfoThread::navigationPathReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testConnectionStateSignal()
{
    QSignalSpy connectionSpy(m_thread, &InfoThread::connectionStateChanged);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestInfoThread::testThreadLifecycle()
{
    QVERIFY(!m_thread->isRunning());

    m_thread->start();
    QThread::msleep(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isRunning());
}
