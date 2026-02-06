// ==================== DEPRECATED ====================
// 此测试文件已废弃，对应的源文件 navstatusthread.h/cpp 已废弃
// 废弃原因：已被 NavigationActionThread 替代
// 保留原因：作为工作量证明保留
// ================================================

#ifndef TESTNAVSTATUSTHREAD_H
#define TESTNAVSTATUSTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "navstatusthread.h"

class TestNavStatusThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testInitialize();
    void testStart();
    void testStop();
    void testNavigationStatusSignal();
    void testNavigationPathSignal();
    void testConnectionStateSignal();
    void testThreadLifecycle();

private:
    NavStatusThread* m_thread;
};

#endif
