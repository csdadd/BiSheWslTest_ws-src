#include "mapwidget.h"
#include <QDebug>
#include <cmath>
#include <QtMath>

// 构造函数：初始化地图显示控件
MapWidget::MapWidget(QWidget* parent)
    : QGraphicsView(parent)
    , m_scene(nullptr)
    , m_mapPixmapItem(nullptr)
    , m_robotItem(nullptr)
    , m_robotIconItem(nullptr)
    , m_resolution(0.0)
    , m_originX(0.0)
    , m_originY(0.0)
    , m_zoomLevel(1.0)
    , m_isDragging(false)
    , m_dragButton(Qt::NoButton)
    , m_zoomAnimation(nullptr)
    , m_scrollAnimation(nullptr)
{
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);

    setDragMode(QGraphicsView::NoDrag);
    setRenderHint(QPainter::Antialiasing);
    setRenderHint(QPainter::SmoothPixmapTransform);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // 初始化标记项对象池
    m_markerPool.reserve(MARKER_POOL_SIZE);
    m_markerPoolUsed.reserve(MARKER_POOL_SIZE);
    for (int i = 0; i < MARKER_POOL_SIZE; ++i) {
        QGraphicsEllipseItem* item = m_scene->addEllipse(
            -5, -5, 10, 10,
            QPen(Qt::black),
            QBrush(Qt::black)
        );
        item->setVisible(false);
        m_markerPool.append(item);
        m_markerPoolUsed.append(false);
    }
}

// 析构函数：清理资源
// QGraphicsScene 会自动清理所有 QGraphicsItem（通过父子关系）
MapWidget::~MapWidget()
{
    if (m_zoomAnimation) {
        m_zoomAnimation->stop();
    }

    if (m_scrollAnimation) {
        m_scrollAnimation->stop();
    }
}

// 设置地图图像：加载地图并设置分辨率和原点坐标
// image: 地图图像
// resolution: 地图分辨率（米/像素）
// originX: 地图原点X坐标
// originY: 地图原点Y坐标
void MapWidget::setMapImage(const QImage& image, double resolution, double originX, double originY)
{
    if (image.isNull()) {
        qWarning() << "[MapWidget] 设置地图失败 - 图像为空";
        return;
    }

    if (resolution <= 0) {
        qWarning() << "[MapWidget] 设置地图失败 - 分辨率无效:" << resolution;
        return;
    }

    if (!std::isfinite(originX) || !std::isfinite(originY)) {
        qWarning() << "[MapWidget] 设置地图失败 - 原点非有限值 originX:" << originX << "originY:" << originY;
        return;
    }

    m_resolution = resolution;
    m_originX = originX;
    m_originY = originY;

    if (m_mapPixmapItem) {
        m_scene->removeItem(m_mapPixmapItem);
        delete m_mapPixmapItem;
        m_mapPixmapItem = nullptr;
    }

    QPixmap pixmap = QPixmap::fromImage(image);
    if (pixmap.isNull()) {
        qWarning() << "[MapWidget] 设置地图失败 - 图像转换为像素图失败";
        return;
    }

    m_mapPixmapItem = m_scene->addPixmap(pixmap);
    m_mapPixmapItem->setZValue(0);
    m_mapPixmapItem->setCacheMode(QGraphicsItem::DeviceCoordinateCache);

    m_scene->setSceneRect(0, 0, image.width(), image.height());

    qDebug() << "[MapWidget] 地图设置成功 - 尺寸:" << image.width() << "x" << image.height()
             << "分辨率:" << m_resolution << "原点:(" << m_originX << "," << m_originY << ")";
}

// 更新机器人位姿：在地图上显示机器人的位置和方向
// x: 机器人X坐标（米）
// y: 机器人Y坐标（米）
// yaw: 机器人偏航角（弧度）
void MapWidget::updateRobotPose(double x, double y, double yaw)
{
    if (!m_mapPixmapItem) {
        qWarning() << "[MapWidget] 更新机器人位姿失败 - 地图未设置";
        return;
    }

    if (m_resolution <= 0) {
        qWarning() << "[MapWidget] 更新机器人位姿失败 - 分辨率无效:" << m_resolution;
        return;
    }

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        qWarning() << "[MapWidget] 更新机器人位姿失败 - 坐标非有限值 x:" << x << "y:" << y << "yaw:" << yaw;
        return;
    }

    QPointF imagePos = MapConverter::mapToImage(x, y, m_resolution, m_originX, m_originY);

    if (!m_robotIcon.isNull()) {
        if (m_robotIconItem) {
            m_robotIconItem->setPos(imagePos);
            m_robotIconItem->setRotation(qRadiansToDegrees(yaw));
        } else {
            m_robotIconItem = m_scene->addPixmap(m_robotIcon);
            m_robotIconItem->setPos(imagePos);
            m_robotIconItem->setRotation(qRadiansToDegrees(yaw));
            m_robotIconItem->setOffset(-m_robotIcon.width() / 2, -m_robotIcon.height() / 2);
            m_robotIconItem->setZValue(1);
        }

        if (m_robotItem) {
            m_scene->removeItem(m_robotItem);
            delete m_robotItem;
            m_robotItem = nullptr;
        }
    } else {
        double robotSize = 0.5 / m_resolution;
        QPolygonF robotPolygon = MapConverter::createRobotPolygon(
            imagePos.x(), imagePos.y(), yaw, robotSize);

        if (m_robotItem) {
            m_robotItem->setPolygon(robotPolygon);
        } else {
            m_robotItem = m_scene->addPolygon(robotPolygon);
            m_robotItem->setBrush(QBrush(QColor(255, 0, 0)));
            m_robotItem->setPen(QPen(QColor(0, 0, 0), 2));
            m_robotItem->setZValue(1);
        }

        if (m_robotIconItem) {
            m_scene->removeItem(m_robotIconItem);
            delete m_robotIconItem;
            m_robotIconItem = nullptr;
        }
    }
}

// 清除机器人位姿：从地图上移除机器人显示
void MapWidget::clearRobotPose()
{
    if (m_robotItem) {
        m_scene->removeItem(m_robotItem);
        delete m_robotItem;
        m_robotItem = nullptr;
    }
}

// 设置缩放级别：设置地图的缩放级别
// level: 目标缩放级别（1.0 = 原始大小）
void MapWidget::setZoomLevel(double level)
{
    if (level <= 0) {
        return;
    }

    double factor = level / m_zoomLevel;
    scale(factor, factor);
    m_zoomLevel = level;

    emit zoomChanged(m_zoomLevel);
}

double MapWidget::getZoomLevel() const
{
    return m_zoomLevel;
}

// 将视图中心定位到机器人：将视图中心定位到机器人图形项的位置
void MapWidget::centerOnRobot()
{
    if (m_robotItem) {
        centerOn(m_robotItem);
    }
}

// 将视图中心定位到指定位置：将视图中心定位到指定的地图坐标位置
// x: 目标位置X坐标（米）
// y: 目标位置Y坐标（米）
void MapWidget::centerOnPosition(double x, double y)
{
    QPointF imagePos = MapConverter::mapToImage(x, y, m_resolution, m_originX, m_originY);
    centerOn(imagePos);
}

void MapWidget::wheelEvent(QWheelEvent* event)
{
    double angle = event->angleDelta().y();
    double factor = 1.0 + angle / 1200.0;

    scale(factor, factor);
    m_zoomLevel *= factor;

    emit zoomChanged(m_zoomLevel);

    QGraphicsView::wheelEvent(event);
}

void MapWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
        m_isDragging = true;
        m_dragButton = event->button();
        m_lastMousePos = event->pos();
        setDragMode(QGraphicsView::ScrollHandDrag);
    } else if (event->button() == Qt::RightButton) {
        QPointF scenePos = mapToScene(event->pos());
        QPointF mapPos = MapConverter::imageToMap(
            scenePos.x(), scenePos.y(), m_resolution, m_originX, m_originY);
        emit mapClicked(mapPos.x(), mapPos.y());
    }

    QGraphicsView::mousePressEvent(event);
}

void MapWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (m_isDragging) {
        QPointF delta = event->pos() - m_lastMousePos;
        translate(delta.x(), delta.y());
        m_lastMousePos = event->pos();
    }

    QGraphicsView::mouseMoveEvent(event);
}

void MapWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == m_dragButton) {
        m_isDragging = false;
        m_dragButton = Qt::NoButton;
        setDragMode(QGraphicsView::NoDrag);

        animateScrollToCenter();
    }

    QGraphicsView::mouseReleaseEvent(event);
}

// 平滑滚动到中心：使用动画效果平滑地滚动视图到中心位置
void MapWidget::animateScrollToCenter()
{
    if (!horizontalScrollBar() || !verticalScrollBar()) {
        return;
    }

    if (m_scrollAnimation) {
        m_scrollAnimation->stop();
        delete m_scrollAnimation;
    }

    m_scrollAnimation = new QPropertyAnimation(static_cast<QObject*>(horizontalScrollBar()), QByteArrayLiteral("value"));
    m_scrollAnimation->setDuration(200);
    m_scrollAnimation->setEasingCurve(QEasingCurve::OutCubic);
    m_scrollAnimation->start(QAbstractAnimation::DeleteWhenStopped);
}

// 缩放动画：使用动画效果平滑地缩放地图
// targetLevel: 目标缩放级别（1.0 = 原始大小）
void MapWidget::animateZoom(double targetLevel)
{
    if (targetLevel <= 0) {
        return;
    }

    if (m_zoomAnimation) {
        m_zoomAnimation->stop();
        delete m_zoomAnimation;
    }

    m_zoomAnimation = new QPropertyAnimation(this, QByteArrayLiteral("zoomLevel"));
    m_zoomAnimation->setDuration(300);
    m_zoomAnimation->setStartValue(m_zoomLevel);
    m_zoomAnimation->setEndValue(targetLevel);
    m_zoomAnimation->setEasingCurve(QEasingCurve::InOutQuad);
    m_zoomAnimation->start(QAbstractAnimation::DeleteWhenStopped);
}

// 键盘事件处理：支持键盘快捷键进行地图操作
void MapWidget::keyPressEvent(QKeyEvent* event)
{
    switch (event->key()) {
        case Qt::Key_Plus:
        case Qt::Key_Equal:
            animateZoom(m_zoomLevel * 1.2);
            break;

        case Qt::Key_Minus:
            animateZoom(m_zoomLevel / 1.2);
            break;

        case Qt::Key_0:
            animateZoom(1.0);
            break;

        case Qt::Key_R:
            centerOnRobot();
            break;

        case Qt::Key_Up:
            translate(0, -50);
            break;

        case Qt::Key_Down:
            translate(0, 50);
            break;

        case Qt::Key_Left:
            translate(-50, 0);
            break;

        case Qt::Key_Right:
            translate(50, 0);
            break;

        default:
            QGraphicsView::keyPressEvent(event);
            break;
    }
}

void MapWidget::setRobotIcon(const QPixmap& icon)
{
    if (icon.isNull()) {
        qWarning() << "[MapWidget] 设置机器人图标失败 - 图标为空";
        return;
    }

    m_robotIcon = icon;

    if (m_robotIconItem) {
        m_scene->removeItem(m_robotIconItem);
        delete m_robotIconItem;
        m_robotIconItem = nullptr;
    }

    qDebug() << "[MapWidget] 设置机器人图标成功 - 尺寸:" << icon.width() << "x" << icon.height();
}

// 从对象池获取标记项
QGraphicsEllipseItem* MapWidget::acquireMarkerItem()
{
    for (int i = 0; i < m_markerPool.size(); ++i) {
        if (!m_markerPoolUsed[i]) {
            m_markerPoolUsed[i] = true;
            m_markerPool[i]->setVisible(true);
            return m_markerPool[i];
        }
    }
    // 池已满，创建新项（备用方案）
    QGraphicsEllipseItem* item = m_scene->addEllipse(
        -5, -5, 10, 10,
        QPen(Qt::black),
        QBrush(Qt::black)
    );
    m_markerPool.append(item);
    m_markerPoolUsed.append(true);
    return item;
}

// 释放标记项回对象池
void MapWidget::releaseMarkerItem(QGraphicsEllipseItem* item)
{
    int index = m_markerPool.indexOf(item);
    if (index >= 0) {
        m_markerPoolUsed[index] = false;
        item->setVisible(false);
        item->setToolTip("");
    }
}

void MapWidget::addMarker(const MapMarker& marker)
{
    if (!m_mapPixmapItem) {
        qWarning() << "[MapWidget] 添加标记点失败 - 地图未设置";
        return;
    }

    if (m_resolution <= 0) {
        qWarning() << "[MapWidget] 添加标记点失败 - 分辨率无效:" << m_resolution;
        return;
    }

    m_markerManager.addMarker(marker);

    QPointF imagePos = MapConverter::mapToImage(
        marker.position.x(), marker.position.y(), m_resolution, m_originX, m_originY);

    // 从对象池获取标记项
    QGraphicsEllipseItem* markerItem = acquireMarkerItem();
    markerItem->setPen(QPen(marker.color));
    markerItem->setBrush(QBrush(marker.color));
    markerItem->setPos(imagePos);
    markerItem->setZValue(2);
    markerItem->setToolTip(marker.name + ": " + marker.description);

    m_markerItems.append(markerItem);
    qDebug() << "[MapWidget] 添加标记点到场景成功 - 名称:" << marker.name;
}

void MapWidget::removeMarker(const QString& name)
{
    m_markerManager.removeMarker(name);

    for (int i = 0; i < m_markerItems.size(); ++i) {
        if (m_markerItems[i]->toolTip().startsWith(name + ":")) {
            releaseMarkerItem(m_markerItems[i]);
            m_markerItems.removeAt(i);
            qDebug() << "[MapWidget] 从场景中删除标记点成功 - 名称:" << name;
            return;
        }
    }
}

void MapWidget::clearMarkers()
{
    for (QGraphicsEllipseItem* item : m_markerItems) {
        releaseMarkerItem(item);
    }
    m_markerItems.clear();
    m_markerManager.clear();
    qDebug() << "[MapWidget] 清除所有标记点";
}

void MapWidget::addStatusIndicator(const StatusIndicator& indicator)
{
    if (!m_mapPixmapItem) {
        qWarning() << "[MapWidget] 添加状态指示器失败 - 地图未设置";
        return;
    }

    if (m_resolution <= 0) {
        qWarning() << "[MapWidget] 添加状态指示器失败 - 分辨率无效:" << m_resolution;
        return;
    }

    m_statusManager.addIndicator(indicator);

    QPointF imagePos = MapConverter::mapToImage(
        indicator.position.x(), indicator.position.y(), m_resolution, m_originX, m_originY);

    QGraphicsTextItem* statusItem = m_scene->addText(indicator.message);
    statusItem->setPos(imagePos);
    statusItem->setZValue(3);

    switch (indicator.type) {
        case StatusType::Info:
            statusItem->setDefaultTextColor(Qt::blue);
            break;
        case StatusType::Warning:
            statusItem->setDefaultTextColor(QColor(255, 165, 0));
            break;
        case StatusType::Error:
            statusItem->setDefaultTextColor(Qt::red);
            break;
    }

    m_statusItems.append(statusItem);
    qDebug() << "[MapWidget] 添加状态指示器到场景成功 - 类型:" << static_cast<int>(indicator.type) << "消息:" << indicator.message;
}

void MapWidget::clearStatusIndicators()
{
    for (QGraphicsTextItem* item : m_statusItems) {
        m_scene->removeItem(item);
        delete item;
    }
    m_statusItems.clear();
    m_statusManager.clear();
    qDebug() << "[MapWidget] 清除所有状态指示器";
}
