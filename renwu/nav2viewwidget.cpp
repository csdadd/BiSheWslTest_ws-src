#include "nav2viewwidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <functional>

Nav2ViewWidget::Nav2ViewWidget(const std::string& map_yaml_path,
                                rclcpp::Node::SharedPtr node,
                                QWidget* parent)
    : QWidget(parent)
    , node_(node)
    , map_resolution_(0.05)
    , map_origin_x_(0.0)
    , map_origin_y_(0.0)
    , map_loaded_(false)
    , robot_x_(0.0)
    , robot_y_(0.0)
    , robot_yaw_(0.0)
    , robot_pose_received_(false)
    , robot_length_(0.5)
    , robot_width_(0.4)
    , goal_x_(0.0)
    , goal_y_(0.0)
    , goal_yaw_(0.0)
    , goal_pose_received_(false)
    , mouse_dragging_(false)
{
    if (!node_) {
        qWarning() << "ROS2 node is null!";
        return;
    }

    if (!loadMapFromYaml(map_yaml_path)) {
        qWarning() << "Failed to load map from:" << QString::fromStdString(map_yaml_path);
        return;
    }

    plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&Nav2ViewWidget::planCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&Nav2ViewWidget::amclPoseCallback, this, std::placeholders::_1));

    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&Nav2ViewWidget::goalPoseCallback, this, std::placeholders::_1));

    goal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    connect(this, &Nav2ViewWidget::dataUpdated, this, [this]() { this->update(); });
}

void Nav2ViewWidget::setRobotSize(double length, double width) {
    robot_length_ = length;
    robot_width_ = width;
}

bool Nav2ViewWidget::loadMapFromYaml(const std::string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (!config["image"]) {
            qWarning() << "YAML missing 'image' field";
            return false;
        }

        std::string image_name = config["image"].as<std::string>();

        std::string yaml_dir = yaml_path.substr(0, yaml_path.find_last_of("/\\"));
        std::string image_path = yaml_dir + "/" + image_name;

        if (!map_image_.load(QString::fromStdString(image_path))) {
            qWarning() << "Failed to load image:" << QString::fromStdString(image_path);
            return false;
        }

        if (config["resolution"]) {
            map_resolution_ = config["resolution"].as<double>();
        }

        if (config["origin"]) {
            auto origin = config["origin"];
            if (origin.IsSequence() && origin.size() >= 2) {
                map_origin_x_ = origin[0].as<double>();
                map_origin_y_ = origin[1].as<double>();
            }
        }

        setFixedSize(map_image_.width(), map_image_.height());
        map_loaded_ = true;
        return true;

    } catch (const YAML::Exception& e) {
        qWarning() << "YAML parsing error:" << e.what();
        return false;
    }
}

QPointF Nav2ViewWidget::mapToQt(double x, double y) const {
    double qt_x = (x - map_origin_x_) / map_resolution_;
    double qt_y = map_image_.height() - (y - map_origin_y_) / map_resolution_;
    return QPointF(qt_x, qt_y);
}

void Nav2ViewWidget::qtToMap(const QPointF& point, double& x, double& y) const {
    x = map_origin_x_ + point.x() * map_resolution_;
    y = map_origin_y_ + (map_image_.height() - point.y()) * map_resolution_;
}

void Nav2ViewWidget::planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_points_.clear();
    for (const auto& pose_stamped : msg->poses) {
        QPointF pt = mapToQt(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        path_points_.push_back(pt);
    }
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = std::atan2(
        2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
        1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z)
    );
    robot_pose_received_ = true;
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_yaw_ = std::atan2(
        2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
               msg->pose.orientation.x * msg->pose.orientation.y),
        1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                      msg->pose.orientation.z * msg->pose.orientation.z)
    );
    goal_pose_received_ = true;
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (map_loaded_) {
        painter.drawImage(0, 0, map_image_);
    }

    if (!path_points_.empty()) {
        painter.setPen(QPen(QColor(0, 0, 255), 2));
        for (size_t i = 1; i < path_points_.size(); ++i) {
            painter.drawLine(path_points_[i - 1], path_points_[i]);
        }
    }

    if (robot_pose_received_) {
        QPointF center = mapToQt(robot_x_, robot_y_);
        double pixel_length = robot_length_ / map_resolution_;
        double pixel_width = robot_width_ / map_resolution_;

        QTransform transform;
        transform.translate(center.x(), center.y());
        transform.rotate(-robot_yaw_ * 180.0 / M_PI);

        QRectF rect(-pixel_length / 2, -pixel_width / 2, pixel_length, pixel_width);
        QPolygonF robot_rect = transform.map(rect);
        painter.setBrush(QColor(0, 200, 0));
        painter.setPen(Qt::black);
        painter.drawPolygon(robot_rect);

        QPolygonF triangle;
        double arrow_size = pixel_length / 3;
        triangle << QPointF(pixel_length / 2, 0)
                 << QPointF(pixel_length / 2 - arrow_size, -arrow_size / 2)
                 << QPointF(pixel_length / 2 - arrow_size, arrow_size / 2);
        QPolygonF triangle_transformed = transform.map(triangle);
        painter.drawPolygon(triangle_transformed);
    }

    if (goal_pose_received_) {
        QPointF center = mapToQt(goal_x_, goal_y_);
        double arrow_size = 20.0;

        QTransform transform;
        transform.translate(center.x(), center.y());
        transform.rotate(-goal_yaw_ * 180.0 / M_PI);

        QPolygonF arrow;
        arrow << QPointF(arrow_size / 2, 0)
              << QPointF(-arrow_size / 2, -arrow_size / 3)
              << QPointF(-arrow_size / 2, arrow_size / 3);

        QPolygonF arrow_transformed = transform.map(arrow);
        painter.setBrush(QColor(0, 0, 255));
        painter.setPen(Qt::black);
        painter.drawPolygon(arrow_transformed);
    }

    if (mouse_dragging_) {
        double drag_yaw = std::atan2(
            mouse_press_pos_.y() - mouse_current_pos_.y(),
            mouse_current_pos_.x() - mouse_press_pos_.x()
        );
        double arrow_size = 20.0;

        QTransform transform;
        transform.translate(mouse_press_pos_.x(), mouse_press_pos_.y());
        transform.rotate(-drag_yaw * 180.0 / M_PI);

        QPolygonF arrow;
        arrow << QPointF(arrow_size / 2, 0)
              << QPointF(-arrow_size / 2, -arrow_size / 3)
              << QPointF(-arrow_size / 2, arrow_size / 3);

        QPolygonF arrow_transformed = transform.map(arrow);
        painter.setBrush(QColor(0, 0, 255, 150));
        painter.setPen(Qt::black);
        painter.drawPolygon(arrow_transformed);

        painter.setPen(QPen(QColor(0, 0, 255, 150), 1, Qt::DashLine));
        painter.drawLine(mouse_press_pos_, mouse_current_pos_);
    }
}

void Nav2ViewWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        mouse_dragging_ = true;
        mouse_press_pos_ = event->pos();
        mouse_current_pos_ = event->pos();
        update();
    }
}

void Nav2ViewWidget::mouseMoveEvent(QMouseEvent* event) {
    if (mouse_dragging_) {
        mouse_current_pos_ = event->pos();
        update();
    }
}

void Nav2ViewWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && mouse_dragging_) {
        mouse_dragging_ = false;

        double x, y;
        qtToMap(mouse_press_pos_, x, y);

        double yaw = std::atan2(
            mouse_press_pos_.y() - mouse_current_pos_.y(),
            mouse_current_pos_.x() - mouse_press_pos_.x()
        );

        // 存储目标但不发布
        goal_x_ = x;
        goal_y_ = y;
        goal_yaw_ = yaw;
        goal_pose_received_ = true;

        // 发射预览信号（不发布到话题）
        Q_EMIT goalPosePreview(x, y, yaw);
        update();
    }
}

void Nav2ViewWidget::publishCurrentGoal()
{
    if (!goal_pose_received_) {
        return;
    }

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = goal_x_;
    goal_msg.pose.position.y = goal_y_;
    goal_msg.pose.position.z = 0.0;

    double cy = std::cos(goal_yaw_ * 0.5);
    double sy = std::sin(goal_yaw_ * 0.5);
    goal_msg.pose.orientation.w = cy;
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = sy;

    goal_pose_pub_->publish(goal_msg);
}

void Nav2ViewWidget::clearGoal()
{
    goal_x_ = 0.0;
    goal_y_ = 0.0;
    goal_yaw_ = 0.0;
    goal_pose_received_ = false;
}
