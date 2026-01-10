#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QMutexLocker>

template<typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue() = default;

    void enqueue(const T& value)
    {
        QMutexLocker locker(&m_mutex);
        m_queue.enqueue(value);
        m_condition.wakeOne();
    }

    T dequeue()
    {
        QMutexLocker locker(&m_mutex);
        while (m_queue.isEmpty()) {
            m_condition.wait(&m_mutex);
        }
        return m_queue.dequeue();
    }

    bool tryDequeue(T& value, int timeout = 100)
    {
        QMutexLocker locker(&m_mutex);
        if (m_queue.isEmpty()) {
            m_condition.wait(&m_mutex, timeout);
        }
        if (!m_queue.isEmpty()) {
            value = m_queue.dequeue();
            return true;
        }
        return false;
    }

    bool isEmpty() const
    {
        QMutexLocker locker(&m_mutex);
        return m_queue.isEmpty();
    }

    int size() const
    {
        QMutexLocker locker(&m_mutex);
        return m_queue.size();
    }

    void clear()
    {
        QMutexLocker locker(&m_mutex);
        m_queue.clear();
    }

private:
    QQueue<T> m_queue;
    mutable QMutex m_mutex;
    QWaitCondition m_condition;
};

#endif // THREADSAFEQUEUE_H
