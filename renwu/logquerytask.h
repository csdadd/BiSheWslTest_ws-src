#ifndef LOGQUERYTASK_H
#define LOGQUERYTASK_H

#include <QObject>
#include <QRunnable>
#include <QDateTime>
#include <QVector>
#include "logstorageengine.h"

class LogQueryTask : public QObject, public QRunnable
{
    Q_OBJECT

public:
    explicit LogQueryTask(LogStorageEngine* engine,
                         const QDateTime& startTime,
                         const QDateTime& endTime,
                         LogLevel minLevel = LogLevel::DEBUG,
                         const QString& source = QString(),
                         const QString& keyword = QString(),
                         int limit = -1,
                         int offset = 0,
                         QObject* parent = nullptr);
    ~LogQueryTask();

    void run() override;

signals:
    void queryCompleted(const QVector<StorageLogEntry>& results);
    void queryFailed(const QString& error);

private:
    LogStorageEngine* m_engine;
    QDateTime m_startTime;
    QDateTime m_endTime;
    LogLevel m_minLevel;
    QString m_source;
    QString m_keyword;
    int m_limit;
    int m_offset;
};

#endif // LOGQUERYTASK_H
