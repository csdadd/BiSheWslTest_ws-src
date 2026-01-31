#ifndef LOGTHREAD_H
#define LOGTHREAD_H

#include "basethread.h"
#include "threadsafequeue.h"
#include "logstorageengine.h"
#include "loglevel.h"
#include <QFile>
#include <QTextStream>
#include <QTextCodec>
#include <QDateTime>
#include <QMutex>
#include <memory>

struct LogEntry {
    QString message;
    LogLevel level;
    QDateTime timestamp;
    QString source;
    QString category;

    LogEntry() : level(LogLevel::INFO) {}
    LogEntry(const QString& msg, LogLevel lvl, const QDateTime& ts, const QString& src = "", const QString& cat = "")
        : message(msg), level(lvl), timestamp(ts), source(src), category(cat) {}
};

class LogThread : public BaseThread
{
    Q_OBJECT

public:
    explicit LogThread(LogStorageEngine* storageEngine, QObject* parent = nullptr);
    ~LogThread();

    void setLogFilePath(const QString& path);
    QString getLogFilePath() const;

    LogStorageEngine* getStorageEngine() const;

public slots:
    void writeLog(const QString& message, LogLevel level);
    void writeLogEntry(const LogEntry& entry);

signals:
    void logFileChanged(const QString& filePath);

protected:
    void initialize() override;
    void process() override;
    void cleanup() override;

private:
    void writeToFile(const QString& message, LogLevel level, const QDateTime& timestamp, const QString& source = "");
    void rotateLogFile();
    void checkDateRollover();
    QString formatLogMessage(const QString& message, LogLevel level, const QDateTime& timestamp, const QString& source = "");
    void processLogQueue();

private:
    QFile m_logFile;
    QTextStream m_logStream;
    ThreadSafeQueue<LogEntry> m_logQueue;

    QString m_logFilePath;
    QString m_logDirectory;
    qint64 m_maxFileSize;
    int m_maxFileCount;
    QMutex m_fileMutex;
    LogStorageEngine* m_storageEngine;
    QString m_currentLogDate;
    int m_processCount = 0;

    static constexpr qint64 DEFAULT_MAX_FILE_SIZE = 10 * 1024 * 1024;
    static constexpr int DEFAULT_MAX_FILE_COUNT = 5;
};

#endif // LOGTHREAD_H
