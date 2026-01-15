#ifndef LOGSTORAGEENGINE_H
#define LOGSTORAGEENGINE_H

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QVector>
#include <QReadWriteLock>
#include <QSqlDatabase>
#include <QSqlError>

enum StorageLogLevel {
    STORAGE_LOG_DEBUG = 0,
    STORAGE_LOG_INFO = 1,
    STORAGE_LOG_WARNING = 2,
    STORAGE_LOG_ERROR = 3,
    STORAGE_LOG_FATAL = 4
};

struct StorageLogEntry {
    QString message;
    int level;
    QDateTime timestamp;
    QString source;
    QString category;
    QString filePath;
    int lineNumber;

    StorageLogEntry() : level(STORAGE_LOG_INFO), lineNumber(0) {}
    StorageLogEntry(const QString& msg, int lvl, const QDateTime& ts,
                     const QString& src = "", const QString& cat = "",
                     const QString& file = "", int line = 0)
        : message(msg), level(lvl), timestamp(ts), source(src), category(cat),
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
                                        int minLevel = -1,
                                        const QString& source = QString(),
                                        const QString& keyword = QString(),
                                        int limit = -1,
                                        int offset = 0);

    int getLogCount(const QDateTime& startTime = QDateTime(),
                    const QDateTime& endTime = QDateTime(),
                    int minLevel = -1);

    bool clearLogs(const QDateTime& beforeTime = QDateTime());
    bool vacuum();

    QString getLastError() const;

signals:
    void logInserted(int count);
    void logQueryCompleted(int count);
    void errorOccurred(const QString& error);

private:
    bool createTables();
    bool createIndexes();
    QString levelToString(int level);

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
