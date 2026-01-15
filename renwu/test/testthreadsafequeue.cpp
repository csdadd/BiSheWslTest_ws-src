#include "testthreadsafequeue.h"

void TestThreadSafeQueue::initTestCase()
{
}

void TestThreadSafeQueue::cleanupTestCase()
{
}

void TestThreadSafeQueue::init()
{
    m_queue = new ThreadSafeQueue<int>();
}

void TestThreadSafeQueue::cleanup()
{
    delete m_queue;
    m_queue = nullptr;
}

void TestThreadSafeQueue::testEnqueue()
{
    QVERIFY(m_queue->isEmpty());
    m_queue->enqueue(1);
    QVERIFY(!m_queue->isEmpty());
    QCOMPARE(m_queue->size(), 1);
}

void TestThreadSafeQueue::testDequeue()
{
    m_queue->enqueue(1);
    m_queue->enqueue(2);
    m_queue->enqueue(3);

    QCOMPARE(m_queue->dequeue(), 1);
    QCOMPARE(m_queue->dequeue(), 2);
    QCOMPARE(m_queue->dequeue(), 3);
    QVERIFY(m_queue->isEmpty());
}

void TestThreadSafeQueue::testIsEmpty()
{
    QVERIFY(m_queue->isEmpty());
    m_queue->enqueue(1);
    QVERIFY(!m_queue->isEmpty());
    m_queue->dequeue();
    QVERIFY(m_queue->isEmpty());
}

void TestThreadSafeQueue::testSize()
{
    QCOMPARE(m_queue->size(), 0);
    m_queue->enqueue(1);
    QCOMPARE(m_queue->size(), 1);
    m_queue->enqueue(2);
    QCOMPARE(m_queue->size(), 2);
    m_queue->dequeue();
    QCOMPARE(m_queue->size(), 1);
}

void TestThreadSafeQueue::testClear()
{
    m_queue->enqueue(1);
    m_queue->enqueue(2);
    m_queue->enqueue(3);
    QCOMPARE(m_queue->size(), 3);

    m_queue->clear();
    QVERIFY(m_queue->isEmpty());
    QCOMPARE(m_queue->size(), 0);
}

void TestThreadSafeQueue::testTryDequeue()
{
    m_queue->enqueue(1);
    m_queue->enqueue(2);

    int value;
    QVERIFY(m_queue->tryDequeue(value));
    QCOMPARE(value, 1);
    QVERIFY(m_queue->tryDequeue(value));
    QCOMPARE(value, 2);
    QVERIFY(!m_queue->tryDequeue(value));
}

void TestThreadSafeQueue::testTryDequeueWithTimeout()
{
    int value;
    QVERIFY(!m_queue->tryDequeue(value, 100));

    m_queue->enqueue(1);
    QVERIFY(m_queue->tryDequeue(value, 100));
    QCOMPARE(value, 1);
}

void TestThreadSafeQueue::testMultiThreadEnqueueDequeue()
{
    const int itemCount = 1000;
    QFuture<void> enqueueFuture = QtConcurrent::run([this, itemCount]() {
        for (int i = 0; i < itemCount; ++i) {
            m_queue->enqueue(i);
        }
    });

    QFuture<void> dequeueFuture = QtConcurrent::run([this, itemCount]() {
        for (int i = 0; i < itemCount; ++i) {
            m_queue->dequeue();
        }
    });

    enqueueFuture.waitForFinished();
    dequeueFuture.waitForFinished();

    QVERIFY(m_queue->isEmpty());
}

void TestThreadSafeQueue::testThreadSafety()
{
    const int threadCount = 10;
    const int itemsPerThread = 100;
    QList<QFuture<void>> futures;

    for (int i = 0; i < threadCount; ++i) {
        futures.append(QtConcurrent::run([this, itemsPerThread]() {
            for (int j = 0; j < itemsPerThread; ++j) {
                m_queue->enqueue(j);
            }
        }));
    }

    for (auto& future : futures) {
        future.waitForFinished();
    }

    QCOMPARE(m_queue->size(), threadCount * itemsPerThread);
}
