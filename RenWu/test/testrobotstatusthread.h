#ifndef TESTROBOTSTATUSTHREAD_H
#define TESTROBOTSTATUSTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "robotstatusthread.h"

class TestRobotStatusThread : public QObject
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
    void testBatteryStatusSignal();
    void testPositionSignal();
    void testOdometrySignal();
    void testSystemTimeSignal();
    void testDiagnosticsSignal();
    void testConnectionStateSignal();
    void testThreadLifecycle();

private:
    RobotStatusThread* m_thread;
};

#endif
