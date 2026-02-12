#ifndef LOGQUERYTASK_H
#define LOGQUERYTASK_H

#include <QDateTime>
#include <QVector>
#include <QString>
#include "logstorageengine.h"
#include "loglevel.h"

struct LogQueryParams {
    QString dbPath;
    QDateTime startTime;
    QDateTime endTime;
    LogLevel minLevel;
    QString source;
    QString keyword;
    int limit;
    int offset;
};

struct LogQueryResult {
    QVector<StorageLogEntry> results;
    QString errorMessage;
    bool success;

    LogQueryResult() : success(false) {}
};

class LogQueryTask
{
public:
    static LogQueryResult execute(const LogQueryParams& params);
};

#endif // LOGQUERYTASK_H
