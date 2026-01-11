#ifndef LOGTHREAD_H
#define LOGTHREAD_H

#include "basethread.h"
#include "threadsafequeue.h"
#include "logstorageengine.h"
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMutex>

enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,
    LOG_FATAL = 4
};

struct LogEntry {
    QString message;
    int level;
    QDateTime timestamp;
    QString source;
    QString category;

    LogEntry() : level(LOG_INFO) {}
    LogEntry(const QString& msg, int lvl, const QDateTime& ts, const QString& src = "", const QString& cat = "")
        : message(msg), level(lvl), timestamp(ts), source(src), category(cat) {}
};

class LogThread : public BaseThread
{
    Q_OBJECT

public:
    explicit LogThread(QObject* parent = nullptr);
    ~LogThread();

    void setLogFilePath(const QString& path);
    QString getLogFilePath() const;

    LogStorageEngine* getStorageEngine() const;

public slots:
    void writeLog(const QString& message, int level);
    void writeLogEntry(const LogEntry& entry);

signals:
    void logFileChanged(const QString& filePath);

protected:
    void initialize() override;
    void process() override;
    void cleanup() override;

private:
    void writeToFile(const QString& message, int level, const QDateTime& timestamp, const QString& source = "");
    void rotateLogFile();
    QString formatLogMessage(const QString& message, int level, const QDateTime& timestamp, const QString& source = "");
    QString levelToString(int level);
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

    static constexpr qint64 DEFAULT_MAX_FILE_SIZE = 10 * 1024 * 1024;
    static constexpr int DEFAULT_MAX_FILE_COUNT = 5;
};

#endif // LOGTHREAD_H
