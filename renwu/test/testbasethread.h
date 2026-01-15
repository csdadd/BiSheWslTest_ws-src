#ifndef TESTBASETHREAD_H
#define TESTBASETHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QThread>
#include <memory>
#include "basethread.h"

class TestBaseThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testStopThread();
    void testIsThreadRunning();
    void testThreadStartedSignal();
    void testThreadStoppedSignal();
    void testThreadErrorSignal();

private:
    BaseThread* m_thread;
};

#endif
