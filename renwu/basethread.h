#ifndef BASETHREAD_H
#define BASETHREAD_H

#include <cstring>
#include <QThread>
#include <atomic>
#include <QString>
#include <memory>
#include <thread>
#include <rclcpp/executors.hpp>

class BaseThread : public QThread
{
    Q_OBJECT

public:
    explicit BaseThread(QObject* parent = nullptr);
    virtual ~BaseThread();

    void stopThread();
    bool isThreadRunning() const;

signals:
    void logMessage(const QString& message, int level);
    void threadStarted(const QString& threadName);
    void threadStopped(const QString& threadName);
    void threadError(const QString& error);

protected:
    void run() override;
    virtual void initialize() = 0;
    virtual void process() = 0;
    virtual void cleanup() = 0;

protected:
    std::atomic<bool> m_running;
    std::atomic<bool> m_stopped;
    QString m_threadName;
    std::shared_ptr<rclcpp::Executor> m_executor;

public:
    void setExecutor(const std::shared_ptr<rclcpp::Executor>& executor) {
        m_executor = executor;
    }
};

#endif // BASETHREAD_H
