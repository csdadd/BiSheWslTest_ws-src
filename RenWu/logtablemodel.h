#ifndef LOGTABLEMODEL_H
#define LOGTABLEMODEL_H

#include <QAbstractTableModel>
#include <QVector>
#include <QDateTime>
#include "logthread.h"
#include "logstorageengine.h"

class LogTableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    explicit LogTableModel(QObject* parent = nullptr);
    ~LogTableModel();

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    void addLogEntry(const LogEntry& entry);
    void addLogEntries(const QVector<LogEntry>& entries);
    void clearLogs();
    LogEntry getLogEntry(int row) const;

    void setMaxLogs(int maxLogs);
    int getMaxLogs() const;

    void setStorageEngine(LogStorageEngine* engine);
    void loadFromDatabase(const QDateTime& startTime = QDateTime(),
                          const QDateTime& endTime = QDateTime(),
                          int minLevel = -1,
                          const QString& source = QString(),
                          const QString& keyword = QString(),
                          int limit = -1);
    void appendFromDatabase(const QDateTime& startTime = QDateTime(),
                            const QDateTime& endTime = QDateTime(),
                            int minLevel = -1,
                            const QString& source = QString(),
                            const QString& keyword = QString(),
                            int limit = -1);

private:
    void enforceMaxLogs();
    QString levelToString(int level) const;
    LogEntry convertStorageEntry(const StorageLogEntry& storageEntry) const;

private:
    QVector<LogEntry> m_logs;
    int m_maxLogs;
    LogStorageEngine* m_storageEngine;
};

#endif // LOGTABLEMODEL_H
