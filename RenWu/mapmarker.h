#ifndef MAPMARKER_H
#define MAPMARKER_H

#include <QString>
#include <QPointF>
#include <QColor>
#include <QList>

struct MapMarker
{
    QString name;
    QPointF position;
    QColor color;
    QString description;
};

class MapMarkerManager
{
public:
    MapMarkerManager();

    void addMarker(const MapMarker& marker);
    void removeMarker(const QString& name);
    QList<MapMarker> getMarkers() const;
    void clear();
    bool contains(const QString& name) const;
    MapMarker getMarker(const QString& name) const;

private:
    QList<MapMarker> m_markers;
};

#endif
