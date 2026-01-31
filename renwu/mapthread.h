#ifndef MAPTHREAD_H
#define MAPTHREAD_H

#include "basethread.h"
#include "mapconverter.h"
#include <QImage>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapThread : public BaseThread
{
    Q_OBJECT

public:
    explicit MapThread(QObject* parent = nullptr);
    ~MapThread() override;

signals:
    void mapReceived(const QImage& mapImage,
                     double resolution,
                     double originX,
                     double originY);

    void connectionStateChanged(bool connected);

protected:
    void initialize() override;

    void process() override;

    void cleanup() override;

private slots:
    void emitMapUpdate();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void attemptReconnect();

    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_mapSub;
    QImage m_currentMapImage;
    double m_resolution;
    double m_originX;
    double m_originY;
    bool m_mapReceived;
    QTimer* m_updateTimer = nullptr;
    QTimer* m_reconnectTimer = nullptr;
    int m_reconnectCount;
    int m_retryCount = 0;
    int m_successCount = 0;
    static const int MAX_RECONNECT_ATTEMPTS = 10;
};

#endif
