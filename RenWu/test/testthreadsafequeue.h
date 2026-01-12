#ifndef TESTTHREADSAFEQUEUE_H
#define TESTTHREADSAFEQUEUE_H

#include <QtTest/QtTest>
#include <QThread>
#include <QFuture>
#include <QtConcurrent>
#include "threadsafequeue.h"

class TestThreadSafeQueue : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testEnqueue();
    void testDequeue();
    void testIsEmpty();
    void testSize();
    void testClear();
    void testTryDequeue();
    void testTryDequeueWithTimeout();
    void testMultiThreadEnqueueDequeue();
    void testThreadSafety();

private:
    ThreadSafeQueue<int>* m_queue;
};

#endif
