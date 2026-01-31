#include "mapthread.h"
#include <QDebug>

MapThread::MapThread(QObject* parent)
    : BaseThread(parent)
    , m_resolution(0.0)
    , m_originX(0.0)
    , m_originY(0.0)
    , m_mapReceived(false)
    , m_reconnectCount(0)
{
    m_threadName = "MapThread";
    qDebug() << "[MapThread] 构造函数";
}

MapThread::~MapThread()
{
    stopThread();
    wait();
}

void MapThread::initialize()
{
    m_rosNode = std::make_shared<rclcpp::Node>("map_subscriber_node");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable();

    m_mapSub = m_rosNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            this->mapCallback(msg);
        }
    );

    m_mapReceived = false;
    m_reconnectCount = 0;
    m_retryCount = 0;
    m_successCount = 0;
    emit connectionStateChanged(true);

    // 设置父对象为this，确保自动内存管理
    m_updateTimer = new QTimer(this);
    m_updateTimer->setInterval(250);
    connect(m_updateTimer, &QTimer::timeout, this, &MapThread::emitMapUpdate);
    m_updateTimer->start();

    m_reconnectTimer = new QTimer(this);
    m_reconnectTimer->setInterval(5000);
    connect(m_reconnectTimer, &QTimer::timeout, this, &MapThread::attemptReconnect);
}

void MapThread::process()
{
    if (m_rosNode) {
        rclcpp::spin_some(m_rosNode);
    }

    if (!m_mapReceived) {
        m_retryCount++;

        if (m_retryCount % 100 == 0) {
            qWarning() << "[MapThread] 等待地图数据... 重试次数:" << m_retryCount;

            if (!m_reconnectTimer->isActive() && m_reconnectCount < MAX_RECONNECT_ATTEMPTS) {
                m_reconnectTimer->start();
            }
        }
    } else {
        m_successCount++;

        if (m_successCount % 1000 == 0) {
            qDebug() << "[MapThread] 地图连接正常 - 成功接收次数:" << m_successCount;
        }

        if (m_reconnectTimer->isActive()) {
            m_reconnectTimer->stop();
            m_reconnectCount = 0;
        }
    }

    QThread::msleep(10);
}

void MapThread::cleanup()
{
    if (m_updateTimer) {
        m_updateTimer->stop();
        delete m_updateTimer;
        m_updateTimer = nullptr;
    }

    if (m_reconnectTimer) {
        m_reconnectTimer->stop();
        delete m_reconnectTimer;
        m_reconnectTimer = nullptr;
    }

    m_mapSub.reset();
    m_rosNode.reset();
    emit connectionStateChanged(false);
}

void MapThread::emitMapUpdate()
{
    if (!m_currentMapImage.isNull()) {
        emit mapReceived(m_currentMapImage, m_resolution, m_originX, m_originY);
    }
}

void MapThread::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) {
        qWarning() << "[MapThread] 接收到空的地图消息";
        return;
    }

    if (msg->info.width == 0 || msg->info.height == 0) {
        qWarning() << "[MapThread] 接收到尺寸无效的地图 - 宽度:" << msg->info.width << "高度:" << msg->info.height;
        return;
    }

    if (msg->data.empty()) {
        qWarning() << "[MapThread] 接收到数据为空的地图";
        return;
    }

    if (msg->info.resolution <= 0) {
        qWarning() << "[MapThread] 接收到分辨率无效的地图 - 分辨率:" << msg->info.resolution;
        return;
    }

    QImage mapImage = MapConverter::convertToImage(msg);
    if (mapImage.isNull()) {
        qWarning() << "[MapThread] 地图转换为图像失败";
        return;
    }

    m_currentMapImage = mapImage;
    m_resolution = msg->info.resolution;
    m_originX = msg->info.origin.position.x;
    m_originY = msg->info.origin.position.y;
    m_mapReceived = true;

    qDebug() << "[MapThread] 地图更新成功 - 尺寸:" << mapImage.width() << "x" << mapImage.height()
             << "分辨率:" << m_resolution << "原点:(" << m_originX << "," << m_originY << ")";
}

void MapThread::attemptReconnect()
{
    if (m_reconnectCount >= MAX_RECONNECT_ATTEMPTS) {
        qWarning() << "[MapThread] 已达到最大重连次数，停止重连尝试";
        m_reconnectTimer->stop();
        return;
    }

    m_reconnectCount++;

    qWarning() << "[MapThread] 尝试重新连接地图服务 - 重连次数:" << m_reconnectCount << "/" << MAX_RECONNECT_ATTEMPTS;

    if (m_rosNode) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliable();

        m_mapSub.reset();
        m_mapSub = m_rosNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->mapCallback(msg);
            }
        );

        qDebug() << "[MapThread] 地图订阅已重新创建";
    }
}
