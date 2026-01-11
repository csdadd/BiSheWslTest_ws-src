#include "logtablemodel.h"
#include <QColor>

LogTableModel::LogTableModel(QObject* parent)
    : QAbstractTableModel(parent)
    , m_maxLogs(1000)
    , m_storageEngine(nullptr)
{
}

LogTableModel::~LogTableModel()
{
}

int LogTableModel::rowCount(const QModelIndex& parent) const
{
    if (parent.isValid()) {
        return 0;
    }
    return m_logs.size();
}

int LogTableModel::columnCount(const QModelIndex& parent) const
{
    if (parent.isValid()) {
        return 0;
    }
    return 4;
}

QVariant LogTableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() >= m_logs.size() || index.column() >= 4) {
        return QVariant();
    }

    if (role == Qt::DisplayRole) {
        const LogEntry& entry = m_logs[index.row()];

        switch (index.column()) {
        case 0:
            return entry.timestamp.toString("yyyy-MM-dd hh:mm:ss.zzz");
        case 1:
            return levelToString(entry.level);
        case 2:
            return entry.source;
        case 3:
            return entry.message;
        default:
            return QVariant();
        }
    }

    if (role == Qt::TextAlignmentRole) {
        if (index.column() == 1) {
            return int(Qt::AlignCenter);
        }
        return int(Qt::AlignLeft | Qt::AlignVCenter);
    }

    if (role == Qt::ForegroundRole) {
        const LogEntry& entry = m_logs[index.row()];
        switch (entry.level) {
        case LOG_DEBUG:
            return QColor(128, 128, 128);
        case LOG_INFO:
            return QColor(0, 0, 0);
        case LOG_WARNING:
            return QColor(255, 165, 0);
        case LOG_ERROR:
            return QColor(255, 0, 0);
        case LOG_FATAL:
            return QColor(139, 0, 0);
        default:
            return QColor(0, 0, 0);
        }
    }

    return QVariant();
}

QVariant LogTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal) {
        switch (section) {
        case 0:
            return "时间";
        case 1:
            return "级别";
        case 2:
            return "来源";
        case 3:
            return "消息";
        default:
            return QVariant();
        }
    }

    return QVariant();
}

void LogTableModel::addLogEntry(const LogEntry& entry)
{
    beginInsertRows(QModelIndex(), m_logs.size(), m_logs.size());
    m_logs.append(entry);
    endInsertRows();

    enforceMaxLogs();
}

void LogTableModel::addLogEntries(const QVector<LogEntry>& entries)
{
    if (entries.isEmpty()) {
        return;
    }

    beginInsertRows(QModelIndex(), m_logs.size(), m_logs.size() + entries.size() - 1);
    m_logs += entries;
    endInsertRows();

    enforceMaxLogs();
}

void LogTableModel::clearLogs()
{
    if (m_logs.isEmpty()) {
        return;
    }

    beginResetModel();
    m_logs.clear();
    endResetModel();
}

LogEntry LogTableModel::getLogEntry(int row) const
{
    if (row < 0 || row >= m_logs.size()) {
        return LogEntry();
    }
    return m_logs[row];
}

void LogTableModel::setMaxLogs(int maxLogs)
{
    m_maxLogs = maxLogs;
    enforceMaxLogs();
}

int LogTableModel::getMaxLogs() const
{
    return m_maxLogs;
}

void LogTableModel::enforceMaxLogs()
{
    if (m_logs.size() > m_maxLogs) {
        int removeCount = m_logs.size() - m_maxLogs;
        beginRemoveRows(QModelIndex(), 0, removeCount - 1);
        m_logs.erase(m_logs.begin(), m_logs.begin() + removeCount);
        endRemoveRows();
    }
}

QString LogTableModel::levelToString(int level) const
{
    switch (level) {
    case LOG_DEBUG:
        return "DEBUG";
    case LOG_INFO:
        return "INFO";
    case LOG_WARNING:
        return "WARN";
    case LOG_ERROR:
        return "ERROR";
    case LOG_FATAL:
        return "FATAL";
    default:
        return "UNKNOWN";
    }
}

void LogTableModel::setStorageEngine(LogStorageEngine* engine)
{
    m_storageEngine = engine;
}

LogEntry LogTableModel::convertStorageEntry(const StorageLogEntry& storageEntry) const
{
    LogEntry entry;
    entry.message = storageEntry.message;
    entry.level = storageEntry.level;
    entry.timestamp = storageEntry.timestamp;
    entry.source = storageEntry.source;
    entry.category = storageEntry.category;
    return entry;
}

void LogTableModel::loadFromDatabase(const QDateTime& startTime,
                                      const QDateTime& endTime,
                                      int minLevel,
                                      const QString& source,
                                      const QString& keyword,
                                      int limit)
{
    if (!m_storageEngine || !m_storageEngine->isInitialized()) {
        qDebug() << "[LogTableModel] Storage engine not initialized";
        return;
    }

    QVector<StorageLogEntry> storageEntries = m_storageEngine->queryLogs(
        startTime, endTime, minLevel, source, keyword, limit
    );

    QVector<LogEntry> entries;
    entries.reserve(storageEntries.size());
    for (const auto& storageEntry : storageEntries) {
        entries.append(convertStorageEntry(storageEntry));
    }

    beginResetModel();
    m_logs.clear();
    m_logs = entries;
    endResetModel();

    qDebug() << "[LogTableModel] Loaded" << entries.size() << "logs from database";
}

void LogTableModel::appendFromDatabase(const QDateTime& startTime,
                                       const QDateTime& endTime,
                                       int minLevel,
                                       const QString& source,
                                       const QString& keyword,
                                       int limit)
{
    if (!m_storageEngine || !m_storageEngine->isInitialized()) {
        qDebug() << "[LogTableModel] Storage engine not initialized";
        return;
    }

    QVector<StorageLogEntry> storageEntries = m_storageEngine->queryLogs(
        startTime, endTime, minLevel, source, keyword, limit
    );

    if (storageEntries.isEmpty()) {
        return;
    }

    QVector<LogEntry> entries;
    entries.reserve(storageEntries.size());
    for (const auto& storageEntry : storageEntries) {
        entries.append(convertStorageEntry(storageEntry));
    }

    beginInsertRows(QModelIndex(), m_logs.size(), m_logs.size() + entries.size() - 1);
    m_logs += entries;
    endInsertRows();

    enforceMaxLogs();

    qDebug() << "[LogTableModel] Appended" << entries.size() << "logs from database";
}
