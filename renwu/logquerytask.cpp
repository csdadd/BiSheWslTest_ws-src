#include "logquerytask.h"
#include <QDebug>

LogQueryTask::LogQueryTask(LogStorageEngine* engine,
                         const QDateTime& startTime,
                         const QDateTime& endTime,
                         LogLevel minLevel,
                         const QString& source,
                         const QString& keyword,
                         int limit,
                         int offset,
                         QObject* parent)
    : QObject(parent)
    , m_engine(engine)
    , m_startTime(startTime)
    , m_endTime(endTime)
    , m_minLevel(minLevel)
    , m_source(source)
    , m_keyword(keyword)
    , m_limit(limit)
    , m_offset(offset)
{
    setAutoDelete(true);
}

LogQueryTask::~LogQueryTask()
{
}

void LogQueryTask::run()
{
    if (!m_engine) {
        QString error = "LogStorageEngine is null";
        qWarning() << "[LogQueryTask]" << error;
        emit queryFailed(error);
        return;
    }

    if (!m_engine->isInitialized()) {
        QString error = "LogStorageEngine is not initialized";
        qWarning() << "[LogQueryTask]" << error;
        emit queryFailed(error);
        return;
    }

    qDebug() << "[LogQueryTask] Starting query:"
             << "startTime:" << m_startTime
             << "endTime:" << m_endTime
             << "minLevel:" << static_cast<int>(m_minLevel)
             << "source:" << m_source
             << "keyword:" << m_keyword
             << "limit:" << m_limit
             << "offset:" << m_offset;

    QVector<StorageLogEntry> results = m_engine->queryLogs(
        m_startTime,
        m_endTime,
        m_minLevel,
        m_source,
        m_keyword,
        m_limit,
        m_offset
    );

    qDebug() << "[LogQueryTask] Query completed, found" << results.size() << "logs";

    emit queryCompleted(results);
}
