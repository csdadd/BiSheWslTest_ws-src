#ifndef ROBOT_MODEL_VIEWER_ROBOT_MODEL_NODE_HPP_
#define ROBOT_MODEL_VIEWER_ROBOT_MODEL_NODE_HPP_

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#endif

#include <QThread>
#include <QPointF>
#include <vector>
#include <memory>

class RobotModelNode : public QThread {
    Q_OBJECT

public:
    RobotModelNode(int argc, char** argv);
    virtual ~RobotModelNode();

    bool init();
    void run();

    std::vector<QPointF> getFootprintPoints() const;
    QPointF getRobotPosition() const;

Q_SIGNALS:
    void footprintUpdated();

private:
    void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    int init_argc_;
    char** init_argv_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<QPointF> footprint_points_;
    QPointF robot_position_;
    bool new_footprint_data_;
};

#endif