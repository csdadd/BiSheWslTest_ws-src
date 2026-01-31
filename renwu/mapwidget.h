#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QPropertyAnimation>
#include <QScrollBar>
#include <QImage>
#include <QPixmap>
#include "mapconverter.h"
#include "mapmarker.h"
#include "statusindicator.h"

class MapWidget : public QGraphicsView
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget* parent = nullptr);
    ~MapWidget() override;

    void setMapImage(const QImage& image, double resolution, double originX, double originY);
    void updateRobotPose(double x, double y, double yaw);
    void clearRobotPose();
    void setZoomLevel(double level);
    double getZoomLevel() const;
    void centerOnRobot();
    void centerOnPosition(double x, double y);
    void animateZoom(double targetLevel);

    void setRobotIcon(const QPixmap& icon);
    void addMarker(const MapMarker& marker);
    void removeMarker(const QString& name);
    void clearMarkers();
    void addStatusIndicator(const StatusIndicator& indicator);
    void clearStatusIndicators();

signals:
    void mapClicked(double x, double y);
    void zoomChanged(double level);

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void animateScrollToCenter();
    QGraphicsScene* m_scene;
    QGraphicsPixmapItem* m_mapPixmapItem;
    QGraphicsPolygonItem* m_robotItem;
    QGraphicsPixmapItem* m_robotIconItem;
    MapMarkerManager m_markerManager;
    QList<QGraphicsEllipseItem*> m_markerItems;
    StatusIndicatorManager m_statusManager;
    QList<QGraphicsTextItem*> m_statusItems;
    double m_resolution;
    double m_originX;
    double m_originY;
    double m_zoomLevel;
    bool m_isDragging;
    QPointF m_lastMousePos;
    Qt::MouseButton m_dragButton;
    QPropertyAnimation* m_zoomAnimation;
    QPropertyAnimation* m_scrollAnimation;
    QPixmap m_robotIcon;

    // 标记项对象池
    QVector<QGraphicsEllipseItem*> m_markerPool;
    QVector<bool> m_markerPoolUsed;
    static constexpr int MARKER_POOL_SIZE = 100;

    QGraphicsEllipseItem* acquireMarkerItem();
    void releaseMarkerItem(QGraphicsEllipseItem* item);
};

#endif
