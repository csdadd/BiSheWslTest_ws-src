#include "logfilterproxymodel.h"

LogFilterProxyModel::LogFilterProxyModel(QObject* parent)
    : QSortFilterProxyModel(parent)
    , m_useRegExp(false)
{
}

LogFilterProxyModel::~LogFilterProxyModel()
{
}

void LogFilterProxyModel::setLogLevelFilter(const QSet<int>& levels)
{
    m_logLevelFilter = levels;
    invalidateFilter();
}

QSet<int> LogFilterProxyModel::logLevelFilter() const
{
    return m_logLevelFilter;
}

void LogFilterProxyModel::setKeywordFilter(const QString& keyword)
{
    m_keywordFilter = keyword;
    invalidateFilter();
}

QString LogFilterProxyModel::keywordFilter() const
{
    return m_keywordFilter;
}

void LogFilterProxyModel::setSourceFilter(const QString& source)
{
    m_sourceFilter = source;
    invalidateFilter();
}

QString LogFilterProxyModel::sourceFilter() const
{
    return m_sourceFilter;
}

void LogFilterProxyModel::setTimeRangeFilter(const QDateTime& start, const QDateTime& end)
{
    m_startTime = start;
    m_endTime = end;
    invalidateFilter();
}

void LogFilterProxyModel::clearTimeRangeFilter()
{
    m_startTime = QDateTime();
    m_endTime = QDateTime();
    invalidateFilter();
}

QDateTime LogFilterProxyModel::startTime() const
{
    return m_startTime;
}

QDateTime LogFilterProxyModel::endTime() const
{
    return m_endTime;
}

void LogFilterProxyModel::setRegExpFilter(const QString& pattern)
{
    m_regExpFilter = pattern;
    invalidateFilter();
}

void LogFilterProxyModel::setUseRegExp(bool use)
{
    m_useRegExp = use;
    invalidateFilter();
}

QString LogFilterProxyModel::regExpFilter() const
{
    return m_regExpFilter;
}

bool LogFilterProxyModel::useRegExp() const
{
    return m_useRegExp;
}

void LogFilterProxyModel::clearAllFilters()
{
    m_logLevelFilter.clear();
    m_keywordFilter.clear();
    m_sourceFilter.clear();
    m_startTime = QDateTime();
    m_endTime = QDateTime();
    m_regExpFilter.clear();
    m_useRegExp = false;
    invalidateFilter();
}

bool LogFilterProxyModel::filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const
{
    QModelIndex levelIndex = sourceModel()->index(sourceRow, 1, sourceParent);
    QModelIndex sourceIndex = sourceModel()->index(sourceRow, 2, sourceParent);
    QModelIndex messageIndex = sourceModel()->index(sourceRow, 3, sourceParent);
    QModelIndex timestampIndex = sourceModel()->index(sourceRow, 0, sourceParent);

    int level = sourceModel()->data(levelIndex, Qt::DisplayRole).toInt();
    QString source = sourceModel()->data(sourceIndex, Qt::DisplayRole).toString();
    QString message = sourceModel()->data(messageIndex, Qt::DisplayRole).toString();
    QDateTime timestamp = sourceModel()->data(timestampIndex, Qt::DisplayRole).toDateTime();

    if (!m_logLevelFilter.isEmpty() && !m_logLevelFilter.contains(level)) {
        return false;
    }

    if (!m_keywordFilter.isEmpty() && !message.contains(m_keywordFilter, Qt::CaseInsensitive)) {
        return false;
    }

    if (!m_sourceFilter.isEmpty() && !source.contains(m_sourceFilter, Qt::CaseInsensitive)) {
        return false;
    }

    if (m_startTime.isValid() && timestamp < m_startTime) {
        return false;
    }

    if (m_endTime.isValid() && timestamp > m_endTime) {
        return false;
    }

    if (m_useRegExp && !m_regExpFilter.isEmpty()) {
        QRegularExpression regExp(m_regExpFilter);
        if (!regExp.isValid()) {
            return false;
        }
        if (!regExp.match(message).hasMatch()) {
            return false;
        }
    }

    return true;
}
