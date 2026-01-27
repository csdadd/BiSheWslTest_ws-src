#pragma once

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Nav2ViewWidget : public QWidget {
    Q_OBJECT

signals:
    void dataUpdated();
    void goalPosePreview(double x, double y, double yaw);

public:
    explicit Nav2ViewWidget(const std::string& map_yaml_path,
                            rclcpp::Node::SharedPtr node,
                            QWidget* parent = nullptr);

    void setRobotSize(double length, double width);
    void publishCurrentGoal();
    void clearGoal();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    bool loadMapFromYaml(const std::string& yaml_path);
    QPointF mapToQt(double x, double y) const;
    void qtToMap(const QPointF& point, double& x, double& y) const;

    void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

    QImage map_image_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    bool map_loaded_;

    std::vector<QPointF> path_points_;

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

    bool mouse_dragging_;
    QPointF mouse_press_pos_;
    QPointF mouse_current_pos_;
};
