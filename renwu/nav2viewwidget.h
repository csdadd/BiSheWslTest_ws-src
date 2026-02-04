#pragma once

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QReadWriteLock>
#include <QTimer>
#include <atomic>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "coordinatetransformer.h"

class Nav2ViewWidget : public QWidget {
    Q_OBJECT

signals:
    void dataUpdated();
    void goalPosePreview(double x, double y, double yaw);
    void mapLoadFailed(const QString& error);
    void mapLoadSucceeded();

public:
    explicit Nav2ViewWidget(const std::string& map_yaml_path,
                            rclcpp::Node::SharedPtr node,
                            QWidget* parent = nullptr);

    void setRobotSize(double length, double width);
    void publishCurrentGoal();
    void clearGoal();

    // 获取当前机器人位置（地图坐标系）
    double getRobotX() const { QReadLocker locker(&data_lock_); return robot_x_; }
    double getRobotY() const { QReadLocker locker(&data_lock_); return robot_y_; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    bool loadMapFromYaml(const std::string& yaml_path);

    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void localPlanCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    QImage map_image_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    bool map_loaded_;

    std::vector<QPointF> path_points_;
    std::vector<QPointF> local_path_points_;
    std::vector<QPointF> scan_points_;

    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool robot_pose_received_;
    double robot_length_;
    double robot_width_;

    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    bool goal_pose_received_;
    bool goal_cleared_manually_ = false;  // 标记目标是否被手动清除

    mutable QReadWriteLock data_lock_;  // 保护ROS回调数据的读写锁

    CoordinateTransformer coord_transformer_;  // 坐标转换器

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  // TF2缓冲区
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // TF2监听器

    bool mouse_dragging_;
    QPointF mouse_press_pos_;
    QPointF mouse_current_pos_;

    // 缓存缩放后的地图图像（用于优化paintEvent性能）
    mutable QImage cached_scaled_map_;
    mutable double cached_scale_ = -1.0;
    mutable QSize cached_widget_size_;

    // 刷新频率控制
    QTimer* update_timer_ = nullptr;
    std::atomic<bool> update_pending_{false};
};
