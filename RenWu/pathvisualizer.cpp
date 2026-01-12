#include "pathvisualizer.h"
#include "mapconverter.h"
#include <QDebug>

PathVisualizer::PathVisualizer(QGraphicsScene* scene, QObject* parent)
    : QObject(parent)
    , m_scene(scene)
    , m_pathItem(nullptr)
    , m_pathColor(Qt::green)
    , m_pathWidth(2.0)
    , m_pathStyle(Qt::SolidLine)
{
    m_pathItem = new QGraphicsPathItem();
    m_pathItem->setPen(QPen(m_pathColor, m_pathWidth, m_pathStyle));
    m_pathItem->setZValue(1);
    m_scene->addItem(m_pathItem);
}

PathVisualizer::~PathVisualizer()
{
    if (m_pathItem) {
        m_scene->removeItem(m_pathItem);
        delete m_pathItem;
    }
}

void PathVisualizer::updatePath(const nav_msgs::msg::Path& path, double resolution, const QPointF& origin)
{
    if (path.poses.empty()) {
        clearPath();
        return;
    }

    QPainterPath painterPath;
    QPointF firstPoint = mapToScene(path.poses[0].pose.position.x, 
                                    path.poses[0].pose.position.y, 
                                    resolution, origin);
    painterPath.moveTo(firstPoint);

    for (size_t i = 1; i < path.poses.size(); ++i) {
        QPointF point = mapToScene(path.poses[i].pose.position.x, 
                                   path.poses[i].pose.position.y, 
                                   resolution, origin);
        painterPath.lineTo(point);
    }

    m_pathItem->setPath(painterPath);
    m_pathItem->setVisible(true);
}

void PathVisualizer::clearPath()
{
    m_pathItem->setPath(QPainterPath());
    m_pathItem->setVisible(false);
}

void PathVisualizer::setPathColor(const QColor& color)
{
    m_pathColor = color;
    m_pathItem->setPen(QPen(m_pathColor, m_pathWidth, m_pathStyle));
}

void PathVisualizer::setPathWidth(double width)
{
    m_pathWidth = width;
    m_pathItem->setPen(QPen(m_pathColor, m_pathWidth, m_pathStyle));
}

void PathVisualizer::setPathStyle(Qt::PenStyle style)
{
    m_pathStyle = style;
    m_pathItem->setPen(QPen(m_pathColor, m_pathWidth, m_pathStyle));
}

QPointF PathVisualizer::mapToScene(double x, double y, double resolution, const QPointF& origin)
{
    return MapConverter::mapToImage(x, y, resolution, origin.x(), origin.y());
}
