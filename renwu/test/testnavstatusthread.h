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
