#include "logstorageengine.h"
#include <QSqlQuery>
#include <QSqlError>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include <QMutexLocker>

LogStorageEngine::LogStorageEngine(QObject* parent)
    : QObject(parent)
    , m_initialized(false)
{
}

LogStorageEngine::~LogStorageEngine()
{
    if (m_database.isOpen()) {
        m_database.close();
    }
}

bool LogStorageEngine::initialize(const QString& dbPath)
{
    QWriteLocker locker(&m_lock);

    if (m_initialized) {
        m_lastError = "Database already initialized";
        return true;
    }

    if (dbPath.isEmpty()) {
        QString dataDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        QDir dir(dataDir);
        if (!dir.exists()) {
            dir.mkpath(dataDir);
        }
        m_dbPath = dataDir + "/logs.db";
    } else {
        m_dbPath = dbPath;
    }

    if (QSqlDatabase::contains("log_connection")) {
        m_database = QSqlDatabase::database("log_connection");
    } else {
        m_database = QSqlDatabase::addDatabase("QSQLITE", "log_connection");
    }

    m_database.setDatabaseName(m_dbPath);

    if (!m_database.open()) {
        m_lastError = QString("Failed to open database: %1").arg(m_database.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!createTables()) {
        m_database.close();
        return false;
    }

    if (!createIndexes()) {
        m_database.close();
        return false;
    }

    m_initialized = true;
    qDebug() << "[LogStorageEngine] Database initialized:" << m_dbPath;
    return true;
}

bool LogStorageEngine::isInitialized() const
{
    QReadLocker locker(&m_lock);
    return m_initialized;
}

bool LogStorageEngine::createTables()
{
    QSqlQuery query(m_database);

    QString createTableSQL = R"(
        CREATE TABLE IF NOT EXISTS logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp INTEGER NOT NULL,
            level INTEGER NOT NULL,
            message TEXT NOT NULL,
            source TEXT,
            category TEXT,
            file_path TEXT,
            line_number INTEGER,
            created_at INTEGER DEFAULT (strftime('%s', 'now'))
        )
    )";

    if (!query.exec(createTableSQL)) {
        m_lastError = QString("Failed to create logs table: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool LogStorageEngine::createIndexes()
{
    QSqlQuery query(m_database);

    QString createTimestampIndex = "CREATE INDEX IF NOT EXISTS idx_timestamp ON logs(timestamp)";
    if (!query.exec(createTimestampIndex)) {
        m_lastError = QString("Failed to create timestamp index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    QString createLevelIndex = "CREATE INDEX IF NOT EXISTS idx_level ON logs(level)";
    if (!query.exec(createLevelIndex)) {
        m_lastError = QString("Failed to create level index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    QString createLevelTimestampIndex = "CREATE INDEX IF NOT EXISTS idx_level_timestamp ON logs(level, timestamp)";
    if (!query.exec(createLevelTimestampIndex)) {
        m_lastError = QString("Failed to create level_timestamp index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    QString createSourceIndex = "CREATE INDEX IF NOT EXISTS idx_source ON logs(source)";
    if (!query.exec(createSourceIndex)) {
        m_lastError = QString("Failed to create source index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool LogStorageEngine::insertLog(const StorageLogEntry& entry)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare(R"(
        INSERT INTO logs (timestamp, level, message, source, category, file_path, line_number)
        VALUES (?, ?, ?, ?, ?, ?, ?)
    )");

    query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
    query.addBindValue(entry.level);
    query.addBindValue(entry.message);
    query.addBindValue(entry.source);
    query.addBindValue(entry.category);
    query.addBindValue(entry.filePath);
    query.addBindValue(entry.lineNumber);

    if (!query.exec()) {
        m_lastError = QString("Failed to insert log: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit logInserted(1);
    return true;
}

bool LogStorageEngine::insertLogs(const QVector<StorageLogEntry>& entries)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    if (entries.isEmpty()) {
        return true;
    }

    QSqlQuery query(m_database);
    query.prepare(R"(
        INSERT INTO logs (timestamp, level, message, source, category, file_path, line_number)
        VALUES (?, ?, ?, ?, ?, ?, ?)
    )");

    if (!m_database.transaction()) {
        m_lastError = QString("Failed to start transaction: %1").arg(m_database.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    int successCount = 0;
    for (const auto& entry : entries) {
        query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
        query.addBindValue(entry.level);
        query.addBindValue(entry.message);
        query.addBindValue(entry.source);
        query.addBindValue(entry.category);
        query.addBindValue(entry.filePath);
        query.addBindValue(entry.lineNumber);

        if (query.exec()) {
            successCount++;
        } else {
            qDebug() << "[LogStorageEngine] Failed to insert log:" << query.lastError().text();
        }
    }

    if (!m_database.commit()) {
        m_lastError = QString("Failed to commit transaction: %1").arg(m_database.lastError().text());
        emit errorOccurred(m_lastError);
        m_database.rollback();
        return false;
    }

    emit logInserted(successCount);
    return true;
}

QVector<StorageLogEntry> LogStorageEngine::queryLogs(const QDateTime& startTime,
                                                        const QDateTime& endTime,
                                                        int minLevel,
                                                        const QString& source,
                                                        const QString& keyword,
                                                        int limit,
                                                        int offset)
{
    QReadLocker locker(&m_lock);

    QVector<StorageLogEntry> results;

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return results;
    }

    QString sql = "SELECT timestamp, level, message, source, category, file_path, line_number FROM logs WHERE 1=1";
    QVector<QVariant> bindValues;

    if (startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(startTime.toMSecsSinceEpoch());
    }

    if (endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(endTime.toMSecsSinceEpoch());
    }

    if (minLevel >= 0) {
        sql += " AND level >= ?";
        bindValues.append(minLevel);
    }

    if (!source.isEmpty()) {
        sql += " AND source = ?";
        bindValues.append(source);
    }

    if (!keyword.isEmpty()) {
        sql += " AND message LIKE ?";
        bindValues.append(QString("%1%").arg(keyword));
    }

    sql += " ORDER BY timestamp DESC";

    if (limit > 0) {
        sql += " LIMIT ?";
        bindValues.append(limit);
    }

    if (offset > 0) {
        sql += " OFFSET ?";
        bindValues.append(offset);
    }

    QSqlQuery query(m_database);
    query.prepare(sql);

    for (const auto& value : bindValues) {
        query.addBindValue(value);
    }

    if (!query.exec()) {
        m_lastError = QString("Failed to query logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return results;
    }

    while (query.next()) {
        StorageLogEntry entry;
        entry.timestamp = QDateTime::fromMSecsSinceEpoch(query.value(0).toLongLong());
        entry.level = query.value(1).toInt();
        entry.message = query.value(2).toString();
        entry.source = query.value(3).toString();
        entry.category = query.value(4).toString();
        entry.filePath = query.value(5).toString();
        entry.lineNumber = query.value(6).toInt();
        results.append(entry);
    }

    emit logQueryCompleted(results.size());
    return results;
}

int LogStorageEngine::getLogCount(const QDateTime& startTime,
                                    const QDateTime& endTime,
                                    int minLevel)
{
    QReadLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return 0;
    }

    QString sql = "SELECT COUNT(*) FROM logs WHERE 1=1";
    QVector<QVariant> bindValues;

    if (startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(startTime.toMSecsSinceEpoch());
    }

    if (endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(endTime.toMSecsSinceEpoch());
    }

    if (minLevel >= 0) {
        sql += " AND level >= ?";
        bindValues.append(minLevel);
    }

    QSqlQuery query(m_database);
    query.prepare(sql);

    for (const auto& value : bindValues) {
        query.addBindValue(value);
    }

    if (!query.exec()) {
        m_lastError = QString("Failed to count logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return 0;
    }

    if (query.next()) {
        return query.value(0).toInt();
    }

    return 0;
}

bool LogStorageEngine::clearLogs(const QDateTime& beforeTime)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);

    if (beforeTime.isValid()) {
        query.prepare("DELETE FROM logs WHERE timestamp < ?");
        query.addBindValue(beforeTime.toMSecsSinceEpoch());
    } else {
        query.prepare("DELETE FROM logs");
    }

    if (!query.exec()) {
        m_lastError = QString("Failed to clear logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[LogStorageEngine] Cleared logs, affected rows:" << query.numRowsAffected();
    return true;
}

bool LogStorageEngine::vacuum()
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);

    if (!query.exec("VACUUM")) {
        m_lastError = QString("Failed to vacuum database: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[LogStorageEngine] Database vacuumed successfully";
    return true;
}

QString LogStorageEngine::getLastError() const
{
    QReadLocker locker(&m_lock);
    return m_lastError;
}

QString LogStorageEngine::levelToString(int level)
{
    switch (level) {
        case STORAGE_LOG_DEBUG:
            return "DEBUG";
        case STORAGE_LOG_INFO:
            return "INFO";
        case STORAGE_LOG_WARNING:
            return "WARN";
        case STORAGE_LOG_ERROR:
            return "ERROR";
        case STORAGE_LOG_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}
