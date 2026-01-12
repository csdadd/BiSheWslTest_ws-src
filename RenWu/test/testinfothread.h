#ifndef TESTINFOTHREAD_H
#define TESTINFOTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "infothread.h"

class TestInfoThread : public QObject
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
    void testSystemStatusSignal();
    void testSystemTimeSignal();
    void testNavigationStatusSignal();
    void testNavigationFeedbackSignal();
    void testNavigationPathSignal();
    void testConnectionStateSignal();
    void testThreadLifecycle();

private:
    InfoThread* m_thread;
};

#endif
