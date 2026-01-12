#ifndef TESTMAPTHREAD_H
#define TESTMAPTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "mapthread.h"

class TestMapThread : public QObject
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
    void testMapReceivedSignal();
    void testConnectionStateSignal();
    void testThreadLifecycle();

private:
    MapThread* m_thread;
};

#endif
