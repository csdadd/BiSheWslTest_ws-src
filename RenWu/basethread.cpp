#include "basethread.h"

BaseThread::BaseThread(QObject* parent)
    : QThread(parent)
    , m_running(false)
    , m_stopped(false)
    , m_threadName("BaseThread")
{
}

BaseThread::~BaseThread()
{
    stopThread();
    wait();
}

void BaseThread::stopThread()
{
    m_stopped = true;
    m_running = false;
}

bool BaseThread::isThreadRunning() const
{
    return m_running;
}

void BaseThread::run()
{
    m_running = true;
    m_stopped = false;

    emit threadStarted(m_threadName);

    try {
        initialize();

        while (!m_stopped && !isInterruptionRequested()) {
            process();
            msleep(10);
        }

        cleanup();

    } catch (const std::exception& e) {
        emit threadError(QString("Exception in %1: %2").arg(m_threadName).arg(e.what()));
    } catch (...) {
        emit threadError(QString("Unknown exception in %1").arg(m_threadName));
    }

    m_running = false;
    emit threadStopped(m_threadName);
}
