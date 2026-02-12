#include "logquerytask.h"
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>
#include <QThread>

LogQueryResult LogQueryTask::execute(const LogQueryParams& params)
{
    LogQueryResult result;
    result.success = false;

    QString connectionName = QString("LogQueryThread_%1").arg(
        reinterpret_cast<quintptr>(QThread::currentThreadId())
    );

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", connectionName);
    db.setDatabaseName(params.dbPath);

    if (!db.open()) {
        result.errorMessage = QString("Failed to open database: %1").arg(db.lastError().text());
        qWarning() << "[LogQueryTask]" << result.errorMessage;
        return result;
    }

    QVector<QVariant> bindValues;
    QString sql = "SELECT timestamp, level, message, source, file_path, line_number FROM logs WHERE 1=1";

    if (params.startTime.isValid()) {
        sql += " AND timestamp >= ?";
        bindValues.append(params.startTime.toMSecsSinceEpoch());
    }

    if (params.endTime.isValid()) {
        sql += " AND timestamp <= ?";
        bindValues.append(params.endTime.toMSecsSinceEpoch());
    }

    if (params.minLevel != LogLevel::DEBUG) {
        sql += " AND level >= ?";
        bindValues.append(static_cast<int>(params.minLevel));
    }

    if (!params.source.isEmpty()) {
        sql += " AND source = ?";
        bindValues.append(params.source);
    }

    if (!params.keyword.isEmpty()) {
        sql += " AND message LIKE ?";
        bindValues.append(QString("%1%").arg(params.keyword));
    }

    sql += " ORDER BY timestamp DESC";

    if (params.limit > 0) {
        sql += " LIMIT ?";
        bindValues.append(params.limit);
    }

    if (params.offset > 0) {
        sql += " OFFSET ?";
        bindValues.append(params.offset);
    }

    QSqlQuery query(db);
    query.prepare(sql);

    for (const auto& value : bindValues) {
        query.addBindValue(value);
    }

    if (!query.exec()) {
        result.errorMessage = QString("Failed to query logs: %1").arg(query.lastError().text());
        qWarning() << "[LogQueryTask]" << result.errorMessage;
        db.close();
        QSqlDatabase::removeDatabase(connectionName);
        return result;
    }

    while (query.next()) {
        StorageLogEntry entry;
        entry.timestamp = QDateTime::fromMSecsSinceEpoch(query.value(0).toLongLong());
        entry.level = static_cast<LogLevel>(query.value(1).toInt());
        entry.message = query.value(2).toString();
        entry.source = query.value(3).toString();
        entry.filePath = query.value(4).toString();
        entry.lineNumber = query.value(5).toInt();
        result.results.append(entry);
    }

    qDebug() << "[LogQueryTask] Query completed, found" << result.results.size() << "logs";

    db.close();
    QSqlDatabase::removeDatabase(connectionName);

    result.success = true;
    return result;
}
