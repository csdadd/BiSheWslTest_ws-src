#ifndef TESTSYSTEMMONITORTHREAD_H
#define TESTSYSTEMMONITORTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "systemmonitorthread.h"

class TestSystemMonitorThread : public QObject
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
    void testLogMessageSignal();
    void testCollisionDetectedSignal();
    void testAnomalyDetectedSignal();
    void testBehaviorTreeLogSignal();
    void testConnectionStateSignal();
    void testOnDiagnosticsReceived();
    void testThreadLifecycle();

private:
    SystemMonitorThread* m_thread;
};

#endif
