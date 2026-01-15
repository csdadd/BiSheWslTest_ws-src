#include "testrobotstatusthread.h"

void TestRobotStatusThread::initTestCase()
{
}

void TestRobotStatusThread::cleanupTestCase()
{
}

void TestRobotStatusThread::init()
{
    m_thread = new RobotStatusThread();
}

void TestRobotStatusThread::cleanup()
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

void TestRobotStatusThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestRobotStatusThread::testInitialize()
{
    QVERIFY(m_thread != nullptr);
}

void TestRobotStatusThread::testStart()
{
    QSignalSpy startedSpy(m_thread, &RobotStatusThread::threadStarted);

    m_thread->start();
    QThread::msleep(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testStop()
{
    QSignalSpy stoppedSpy(m_thread, &RobotStatusThread::threadStopped);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestRobotStatusThread::testBatteryStatusSignal()
{
    QSignalSpy batterySpy(m_thread, &RobotStatusThread::batteryStatusReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testPositionSignal()
{
    QSignalSpy positionSpy(m_thread, &RobotStatusThread::positionReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testOdometrySignal()
{
    QSignalSpy odometrySpy(m_thread, &RobotStatusThread::odometryReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testSystemTimeSignal()
{
    QSignalSpy timeSpy(m_thread, &RobotStatusThread::systemTimeReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testDiagnosticsSignal()
{
    QSignalSpy diagnosticsSpy(m_thread, &RobotStatusThread::diagnosticsReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testConnectionStateSignal()
{
    QSignalSpy connectionSpy(m_thread, &RobotStatusThread::connectionStateChanged);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestRobotStatusThread::testThreadLifecycle()
{
    QVERIFY(!m_thread->isRunning());

    m_thread->start();
    QThread::msleep(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isRunning());
}

#include "testrobotstatusthread.moc"