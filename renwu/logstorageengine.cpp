#include "logstorageengine.h"
#include "logutils.h"
#include <QSqlQuery>
#include <QSqlError>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include <QMutexLocker>
#include <QThread>

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
    if (!m_connectionName.isEmpty()) {
        QSqlDatabase::removeDatabase(m_connectionName);
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

    m_connectionName = QString("LogStorageEngine_%1").arg(
        reinterpret_cast<quintptr>(QThread::currentThreadId())
    );
    m_database = QSqlDatabase::addDatabase("QSQLITE", m_connectionName);

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

    QString createHighFreqTableSQL = R"(
        CREATE TABLE IF NOT EXISTS high_freq_logs (
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

    if (!query.exec(createHighFreqTableSQL)) {
        m_lastError = QString("Failed to create high_freq_logs table: %1").arg(query.lastError().text());
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

    // 高频日志表索引
    QString createHfTimestampIndex = "CREATE INDEX IF NOT EXISTS idx_hf_timestamp ON high_freq_logs(timestamp)";
    if (!query.exec(createHfTimestampIndex)) {
        m_lastError = QString("Failed to create high_freq timestamp index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    QString createHfSourceIndex = "CREATE INDEX IF NOT EXISTS idx_hf_source ON high_freq_logs(source)";
    if (!query.exec(createHfSourceIndex)) {
        m_lastError = QString("Failed to create high_freq source index: %1").arg(query.lastError().text());
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
        VALUES (?, ?, ?, ?, '', ?, ?)
    )");

    query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
    query.addBindValue(static_cast<int>(entry.level));
    query.addBindValue(entry.message);
    query.addBindValue(entry.source);
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
        VALUES (?, ?, ?, ?, '', ?, ?)
    )");

    if (!m_database.transaction()) {
        m_lastError = QString("Failed to start transaction: %1").arg(m_database.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    int successCount = 0;
    for (const auto& entry : entries) {
        query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
        query.addBindValue(static_cast<int>(entry.level));
        query.addBindValue(entry.message);
        query.addBindValue(entry.source);
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
                                                        LogLevel minLevel,
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

    QString sql = "SELECT timestamp, level, message, source, file_path, line_number FROM logs WHERE 1=1";
    QVector<QVariant> bindValues;

    if (startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(startTime.toMSecsSinceEpoch());
    }

    if (endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(endTime.toMSecsSinceEpoch());
    }

    if (minLevel != LogLevel::DEBUG) {
        sql += " AND level >= ?";
        bindValues.append(static_cast<int>(minLevel));
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
        entry.level = static_cast<LogLevel>(query.value(1).toInt());
        entry.message = query.value(2).toString();
        entry.source = query.value(3).toString();
        entry.filePath = query.value(4).toString();
        entry.lineNumber = query.value(5).toInt();
        results.append(entry);
    }

    emit logQueryCompleted(results.size());
    return results;
}

int LogStorageEngine::getLogCount(const QDateTime& startTime,
                                    const QDateTime& endTime,
                                    LogLevel minLevel)
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

    if (minLevel != LogLevel::DEBUG) {
        sql += " AND level >= ?";
        bindValues.append(static_cast<int>(minLevel));
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

bool LogStorageEngine::insertHighFreqLog(const StorageLogEntry& entry)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare(R"(
        INSERT INTO high_freq_logs (timestamp, level, message, source, category, file_path, line_number)
        VALUES (?, ?, ?, ?, '', ?, ?)
    )");

    query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
    query.addBindValue(static_cast<int>(entry.level));
    query.addBindValue(entry.message);
    query.addBindValue(entry.source);
    query.addBindValue(entry.filePath);
    query.addBindValue(entry.lineNumber);

    if (!query.exec()) {
        m_lastError = QString("Failed to insert high freq log: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit logInserted(1);
    return true;
}

bool LogStorageEngine::insertHighFreqLogs(const QVector<StorageLogEntry>& entries)
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
        INSERT INTO high_freq_logs (timestamp, level, message, source, category, file_path, line_number)
        VALUES (?, ?, ?, ?, '', ?, ?)
    )");

    if (!m_database.transaction()) {
        m_lastError = QString("Failed to start transaction: %1").arg(m_database.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    int successCount = 0;
    for (const auto& entry : entries) {
        query.addBindValue(entry.timestamp.toMSecsSinceEpoch());
        query.addBindValue(static_cast<int>(entry.level));
        query.addBindValue(entry.message);
        query.addBindValue(entry.source);
        query.addBindValue(entry.filePath);
        query.addBindValue(entry.lineNumber);

        if (query.exec()) {
            successCount++;
        } else {
            qDebug() << "[LogStorageEngine] Failed to insert high freq log:" << query.lastError().text();
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

QVector<StorageLogEntry> LogStorageEngine::queryHighFreqLogs(const QDateTime& startTime,
                                                             const QDateTime& endTime,
                                                             int limit,
                                                             int offset)
{
    QReadLocker locker(&m_lock);

    QVector<StorageLogEntry> results;

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return results;
    }

    QString sql = "SELECT timestamp, level, message, source, file_path, line_number FROM high_freq_logs WHERE 1=1";
    QVector<QVariant> bindValues;

    if (startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(startTime.toMSecsSinceEpoch());
    }

    if (endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(endTime.toMSecsSinceEpoch());
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
        m_lastError = QString("Failed to query high freq logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return results;
    }

    while (query.next()) {
        StorageLogEntry entry;
        entry.timestamp = QDateTime::fromMSecsSinceEpoch(query.value(0).toLongLong());
        entry.level = static_cast<LogLevel>(query.value(1).toInt());
        entry.message = query.value(2).toString();
        entry.source = query.value(3).toString();
        entry.filePath = query.value(4).toString();
        entry.lineNumber = query.value(5).toInt();
        results.append(entry);
    }

    emit logQueryCompleted(results.size());
    return results;
}

int LogStorageEngine::getHighFreqLogCount(const QDateTime& startTime,
                                          const QDateTime& endTime)
{
    QReadLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return 0;
    }

    QString sql = "SELECT COUNT(*) FROM high_freq_logs WHERE 1=1";
    QVector<QVariant> bindValues;

    if (startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(startTime.toMSecsSinceEpoch());
    }

    if (endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(endTime.toMSecsSinceEpoch());
    }

    QSqlQuery query(m_database);
    query.prepare(sql);

    for (const auto& value : bindValues) {
        query.addBindValue(value);
    }

    if (!query.exec()) {
        m_lastError = QString("Failed to count high freq logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return 0;
    }

    if (query.next()) {
        return query.value(0).toInt();
    }

    return 0;
}

bool LogStorageEngine::clearHighFreqLogs(const QDateTime& beforeTime)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);

    if (beforeTime.isValid()) {
        query.prepare("DELETE FROM high_freq_logs WHERE timestamp < ?");
        query.addBindValue(beforeTime.toMSecsSinceEpoch());
    } else {
        query.prepare("DELETE FROM high_freq_logs");
    }

    if (!query.exec()) {
        m_lastError = QString("Failed to clear high freq logs: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[LogStorageEngine] Cleared high freq logs, affected rows:" << query.numRowsAffected();
    return true;
}
