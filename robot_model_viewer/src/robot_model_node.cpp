#include "robot_model_viewer/robot_model_node.hpp"

using namespace std::placeholders;

RobotModelNode::RobotModelNode(int argc, char** argv)
    : init_argc_(argc), init_argv_(argv), new_footprint_data_(false) {
}

RobotModelNode::~RobotModelNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    wait();
}

bool RobotModelNode::init() {
    rclcpp::init(init_argc_, init_argv_);
    node_ = std::make_shared<rclcpp::Node>("robot_model_viewer");

    if (!rclcpp::ok()) {
        return false;
    }

    footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/local_costmap/published_footprint", 10,
        std::bind(&RobotModelNode::footprintCallback, this, _1));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&RobotModelNode::odomCallback, this, _1));

    start();
    return true;
}

void RobotModelNode::run() {
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }
}

void RobotModelNode::footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    footprint_points_.clear();

    for (const auto& point : msg->polygon.points) {
        footprint_points_.emplace_back(point.x, point.y);
    }

    new_footprint_data_ = true;
    Q_EMIT footprintUpdated();
}

void RobotModelNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_position_.setX(msg->pose.pose.position.x);
    robot_position_.setY(msg->pose.pose.position.y);
}

std::vector<QPointF> RobotModelNode::getFootprintPoints() const {
    return footprint_points_;
}

QPointF RobotModelNode::getRobotPosition() const {
    return robot_position_;
}