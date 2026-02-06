// ==================== DEPRECATED ====================
// 此测试文件已废弃，对应的源文件 mapthread.h/cpp 已废弃
// 废弃原因：项目改用固定地图图片，不再订阅实时地图
// 保留原因：作为工作量证明保留
// ================================================

#include "testmapthread.h"

void TestMapThread::initTestCase()
{
    QSKIP("此测试已废弃：MapThread 已废弃，项目改用固定地图图片");
}

void TestMapThread::cleanupTestCase()
{
}

void TestMapThread::init()
{
    m_thread = new MapThread();
}

void TestMapThread::cleanup()
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

void TestMapThread::testConstructor()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(!m_thread->isThreadRunning());
}

void TestMapThread::testInitialize()
{
    QVERIFY(m_thread != nullptr);
}

void TestMapThread::testStart()
{
    QSignalSpy startedSpy(m_thread, &MapThread::threadStarted);

    m_thread->start();
    QThread::msleep(100);

    QVERIFY(m_thread->isRunning());
    QVERIFY(startedSpy.count() > 0);

    m_thread->stopThread();
    m_thread->wait();
}

void TestMapThread::testStop()
{
    QSignalSpy stoppedSpy(m_thread, &MapThread::threadStopped);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();

    QVERIFY(!m_thread->isRunning());
    QVERIFY(stoppedSpy.count() > 0);
}

void TestMapThread::testMapReceivedSignal()
{
    QSignalSpy mapSpy(m_thread, &MapThread::mapReceived);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestMapThread::testConnectionStateSignal()
{
    QSignalSpy connectionSpy(m_thread, &MapThread::connectionStateChanged);

    m_thread->start();
    QThread::msleep(100);

    m_thread->stopThread();
    m_thread->wait();
}

void TestMapThread::testThreadLifecycle()
{
    QVERIFY(!m_thread->isRunning());

    m_thread->start();
    QThread::msleep(100);
    QVERIFY(m_thread->isRunning());

    m_thread->stopThread();
    m_thread->wait();
    QVERIFY(!m_thread->isRunning());
}

#include "testmapthread.moc"
