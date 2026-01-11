#ifndef MAPCACHE_H
#define MAPCACHE_H

#include <QString>
#include <QMap>
#include <QObject>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapCache : public QObject
{
    Q_OBJECT

public:
    explicit MapCache(int maxSize = 10, QObject* parent = nullptr);

    void add(const QString& key, const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

    nav_msgs::msg::OccupancyGrid::SharedPtr get(const QString& key);

    void remove(const QString& key);

    void clear();

    bool contains(const QString& key) const;

    int size() const;

    int maxSize() const;

    void setMaxSize(int maxSize);

private:
    void evictOldest();

    QMap<QString, nav_msgs::msg::OccupancyGrid::SharedPtr> m_cache;
    QMap<QString, qint64> m_timestamps;
    int m_maxSize;
};

#endif
