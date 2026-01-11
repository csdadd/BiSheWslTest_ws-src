#include "mapcache.h"
#include <QDateTime>
#include <QDebug>
#include <limits>

MapCache::MapCache(int maxSize, QObject* parent)
    : QObject(parent)
    , m_maxSize(maxSize)
{
}

void MapCache::add(const QString& key, const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
    if (!map) {
        qWarning() << "[MapCache] 尝试添加空地图到缓存";
        return;
    }

    if (m_cache.contains(key)) {
        m_cache[key] = map;
        m_timestamps[key] = QDateTime::currentMSecsSinceEpoch();
        qDebug() << "[MapCache] 更新缓存:" << key;
        return;
    }

    if (m_cache.size() >= m_maxSize) {
        evictOldest();
    }

    m_cache[key] = map;
    m_timestamps[key] = QDateTime::currentMSecsSinceEpoch();
    qDebug() << "[MapCache] 添加地图到缓存:" << key << "当前缓存大小:" << m_cache.size();
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapCache::get(const QString& key)
{
    if (m_cache.contains(key)) {
        m_timestamps[key] = QDateTime::currentMSecsSinceEpoch();
        qDebug() << "[MapCache] 从缓存获取地图:" << key;
        return m_cache[key];
    }

    qWarning() << "[MapCache] 缓存中未找到地图:" << key;
    return nullptr;
}

void MapCache::remove(const QString& key)
{
    if (m_cache.remove(key) > 0) {
        m_timestamps.remove(key);
        qDebug() << "[MapCache] 从缓存移除地图:" << key;
    }
}

void MapCache::clear()
{
    int count = m_cache.size();
    m_cache.clear();
    m_timestamps.clear();
    qDebug() << "[MapCache] 清空缓存，移除了" << count << "个地图";
}

bool MapCache::contains(const QString& key) const
{
    return m_cache.contains(key);
}

int MapCache::size() const
{
    return m_cache.size();
}

int MapCache::maxSize() const
{
    return m_maxSize;
}

void MapCache::setMaxSize(int maxSize)
{
    if (maxSize <= 0) {
        qWarning() << "[MapCache] 无效的缓存大小:" << maxSize;
        return;
    }

    m_maxSize = maxSize;

    while (m_cache.size() > m_maxSize) {
        evictOldest();
    }

    qDebug() << "[MapCache] 设置缓存大小为:" << m_maxSize;
}

void MapCache::evictOldest()
{
    if (m_cache.isEmpty()) {
        return;
    }

    QString oldestKey;
    qint64 oldestTimestamp = std::numeric_limits<qint64>::max();

    for (auto it = m_timestamps.constBegin(); it != m_timestamps.constEnd(); ++it) {
        if (it.value() < oldestTimestamp) {
            oldestTimestamp = it.value();
            oldestKey = it.key();
        }
    }

    if (!oldestKey.isEmpty()) {
        m_cache.remove(oldestKey);
        m_timestamps.remove(oldestKey);
        qDebug() << "[MapCache] 淘汰最旧的地图:" << oldestKey;
    }
}
