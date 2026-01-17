#include "basethread.h"
#include <rclcpp/rclcpp.hpp>
#include <QDebug>

BaseThread::BaseThread(QObject* parent)
    : QThread(parent)
    , m_running(false)
    , m_stopped(false)
    , m_threadName("BaseThread")
{
    qDebug() << "[BaseThread] 构造函数 - 线程名称:" << m_threadName;
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

    qDebug() << "[BaseThread] 线程开始运行 -" << m_threadName;
    emit threadStarted(m_threadName);

    try {
        initialize();

        std::thread spinThread([this]() {
            while (rclcpp::ok() && !m_stopped) {
                if (m_executor) {
                    m_executor->spin();
                }
            }
        });
        spinThread.detach();

        while (!m_stopped && !isInterruptionRequested()) {
            process();
            msleep(10);
        }

        cleanup();

    } catch (const std::exception& e) {
        qCritical() << "[BaseThread] 异常捕获 -" << m_threadName << "-" << e.what();
        emit threadError(QString("Exception in %1: %2").arg(m_threadName).arg(e.what()));
    } catch (...) {
        qCritical() << "[BaseThread] 未知异常 -" << m_threadName;
        emit threadError(QString("Unknown exception in %1").arg(m_threadName));
    }

    m_running = false;
    qDebug() << "[BaseThread] 线程已停止 -" << m_threadName;
    emit threadStopped(m_threadName);
}
