#ifndef LOGSTORAGEENGINE_H
#define LOGSTORAGEENGINE_H

#include "loglevel.h"
#include <QObject>
#include <QString>
#include <QDateTime>
#include <QVector>
#include <QReadWriteLock>
#include <QSqlDatabase>
#include <QSqlError>

// 使用统一的LogLevel定义
using StorageLogLevel = LogLevel;

struct StorageLogEntry {
    QString message;
    LogLevel level;
    QDateTime timestamp;
    QString source;
    QString filePath;
    int lineNumber;

    StorageLogEntry() : level(LogLevel::INFO), lineNumber(0) {}
    StorageLogEntry(const QString& msg, LogLevel lvl, const QDateTime& ts,
                     const QString& src = "", const QString& file = "", int line = 0)
        : message(msg), level(lvl), timestamp(ts), source(src),
          filePath(file), lineNumber(line) {}
};

Q_DECLARE_METATYPE(StorageLogEntry)
Q_DECLARE_METATYPE(QVector<StorageLogEntry>)

class LogStorageEngine : public QObject
{
    Q_OBJECT

public:
    explicit LogStorageEngine(QObject* parent = nullptr);
    ~LogStorageEngine();

    bool initialize(const QString& dbPath = QString());
    bool isInitialized() const;

    bool insertLog(const StorageLogEntry& entry);
    bool insertLogs(const QVector<StorageLogEntry>& entries);

    QVector<StorageLogEntry> queryLogs(const QDateTime& startTime,
                                        const QDateTime& endTime,
                                        LogLevel minLevel = LogLevel::DEBUG,
                                        const QString& source = QString(),
                                        const QString& keyword = QString(),
                                        int limit = -1,
                                        int offset = 0);

    int getLogCount(const QDateTime& startTime = QDateTime(),
                    const QDateTime& endTime = QDateTime(),
                    LogLevel minLevel = LogLevel::DEBUG);

    bool clearLogs(const QDateTime& beforeTime = QDateTime());
    bool vacuum();

    QString getLastError() const;
    QString getDbPath() const;

    // 高频日志相关方法
    bool insertHighFreqLog(const StorageLogEntry& entry);
    bool insertHighFreqLogs(const QVector<StorageLogEntry>& entries);

    QVector<StorageLogEntry> queryHighFreqLogs(const QDateTime& startTime,
                                                const QDateTime& endTime,
                                                int limit = -1,
                                                int offset = 0);
    int getHighFreqLogCount(const QDateTime& startTime = QDateTime(),
                            const QDateTime& endTime = QDateTime());

    bool clearHighFreqLogs(const QDateTime& beforeTime = QDateTime());

signals:
    void logInserted(int count);
    void logQueryCompleted(int count);
    void errorOccurred(const QString& error);

private:
    bool createTables();
    bool createIndexes();

private:
    QSqlDatabase m_database;
    QString m_connectionName;
    QString m_dbPath;
    bool m_initialized;
    mutable QReadWriteLock m_lock;
    QString m_lastError;
    static constexpr int BATCH_INSERT_SIZE = 100;
};

#endif // LOGSTORAGEENGINE_H
