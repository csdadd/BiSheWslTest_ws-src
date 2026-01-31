#include "nav2viewwidget.h"
#include "geometryutils.h"
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
    , goal_cleared_manually_(false)
    , mouse_dragging_(false)
    , coord_transformer_(0.05, 0.0, 0.0, 0)
{
    if (!node_) {
        qWarning() << "ROS2 node is null!";
        return;
    }

    if (!loadMapFromYaml(map_yaml_path)) {
        QString error_msg = QString("Failed to load map from: %1").arg(QString::fromStdString(map_yaml_path));
        qWarning() << error_msg;
        map_loaded_ = false;
        Q_EMIT mapLoadFailed(error_msg);
        return;
    }
    Q_EMIT mapLoadSucceeded();

    plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&Nav2ViewWidget::planCallback, this, std::placeholders::_1));

    amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&Nav2ViewWidget::amclPoseCallback, this, std::placeholders::_1));

    goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&Nav2ViewWidget::goalPoseCallback, this, std::placeholders::_1));

    goal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    local_plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/local_plan", 10, std::bind(&Nav2ViewWidget::localPlanCallback, this, std::placeholders::_1));

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Nav2ViewWidget::scanCallback, this, std::placeholders::_1));

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

        qDebug() << "[Nav2ViewWidget] 地图加载成功:" << QString::fromStdString(image_path);
        qDebug() << "[Nav2ViewWidget] 地图尺寸:" << map_image_.width() << "x" << map_image_.height();
        qDebug() << "[Nav2ViewWidget] 原始像素格式:" << map_image_.format();

        // 转换为 RGB 格式以确保正确显示
        if (map_image_.format() == QImage::Format_Indexed8) {
            map_image_ = map_image_.convertToFormat(QImage::Format_RGB32);
            qDebug() << "[Nav2ViewWidget] 已转换为 RGB32 格式";
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

        // 初始化坐标转换器
        coord_transformer_ = CoordinateTransformer(map_resolution_, map_origin_x_, map_origin_y_, map_image_.height());

        qDebug() << "[Nav2ViewWidget] 地图分辨率:" << map_resolution_;
        qDebug() << "[Nav2ViewWidget] 地图原点:" << map_origin_x_ << "," << map_origin_y_;

        map_loaded_ = true;
        return true;

    } catch (const YAML::Exception& e) {
        qWarning() << "YAML parsing error:" << e.what();
        return false;
    }
}

void Nav2ViewWidget::planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    {
        QWriteLocker locker(&data_lock_);
        path_points_.clear();
        for (const auto& pose_stamped : msg->poses) {
            QPointF pt = coord_transformer_.mapToQt(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            path_points_.push_back(pt);
        }
    } // 锁在此处释放
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    {
        QWriteLocker locker(&data_lock_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = GeometryUtils::quaternionToYaw(msg->pose.pose.orientation);
        robot_pose_received_ = true;
    } // 锁在此处释放
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 如果用户手动清除了目标，忽略来自话题的更新
    if (goal_cleared_manually_) {
        return;
    }

    {
        QWriteLocker locker(&data_lock_);
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_yaw_ = GeometryUtils::quaternionToYaw(msg->pose.orientation);
        goal_pose_received_ = true;
    } // 锁在此处释放
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::localPlanCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    {
        QWriteLocker locker(&data_lock_);
        local_path_points_.clear();

        std::string frame_id = msg->header.frame_id;

        for (const auto& pose_stamped : msg->poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;

            // odom 坐标系需要转换为 map 坐标系
            // 简化处理：假设 odom 原点是机器人起始位置，使用当前机器人位置作为偏移
            if (frame_id.find("odom") != std::string::npos) {
                x += robot_x_;
                y += robot_y_;
            }
            // 如果已经是 map 坐标系，直接使用原始坐标

            QPointF pt = coord_transformer_.mapToQt(x, y);
            local_path_points_.push_back(pt);
        }
    } // 锁在此处释放
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    {
        QWriteLocker locker(&data_lock_);
        scan_points_.clear();

        std::string frame_id = msg->header.frame_id;
        static int scanCount = 0;
        scanCount++;
        if (scanCount % 10 == 0) {
            qDebug() << "[Nav2ViewWidget] 激光坐标系:" << QString::fromStdString(frame_id)
                     << "机器人位置:" << robot_x_ << robot_y_ << "航向:" << robot_yaw_;
        }

        float angle = msg->angle_min;
        for (const auto& range : msg->ranges) {
            if (range >= msg->range_min && range <= msg->range_max &&
                std::isfinite(range)) {

                // 激光坐标系下的坐标（相对于激光雷达）
                float laser_x = range * std::cos(angle);
                float laser_y = range * std::sin(angle);

                // 转换为地图坐标系：激光 -> 机器人 -> 地图
                // 假设激光雷达与机器人中心重合（或偏移很小）
                double map_x = robot_x_ + laser_x * std::cos(robot_yaw_) - laser_y * std::sin(robot_yaw_);
                double map_y = robot_y_ + laser_x * std::sin(robot_yaw_) + laser_y * std::cos(robot_yaw_);

                QPointF pt = coord_transformer_.mapToQt(map_x, map_y);
                scan_points_.push_back(pt);
            }
            angle += msg->angle_increment;
        }
    } // 锁在此处释放
    Q_EMIT dataUpdated();
}

void Nav2ViewWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QReadLocker locker(&data_lock_);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    ViewTransform vt{1.0, 0.0, 0.0};

    if (map_loaded_) {
        vt = calculateViewTransform(width(), height(), map_image_.width(), map_image_.height());

        // 使用缓存的缩放地图（仅在必要时重新计算）
        QSize current_size(width(), height());
        if (cached_scale_ != vt.scale || cached_widget_size_ != current_size || cached_scaled_map_.isNull()) {
            cached_scaled_map_ = map_image_.scaled(
                map_image_.width() * vt.scale,
                map_image_.height() * vt.scale,
                Qt::KeepAspectRatio,
                Qt::SmoothTransformation
            );
            cached_scale_ = vt.scale;
            cached_widget_size_ = current_size;
        }
        painter.drawImage(QPointF(vt.offset_x, vt.offset_y), cached_scaled_map_);
    }

    if (!path_points_.empty()) {
        painter.setPen(QPen(QColor(0, 0, 255), 2));
        for (size_t i = 1; i < path_points_.size(); ++i) {
            QPointF p1 = path_points_[i - 1] * vt.scale + QPointF(vt.offset_x, vt.offset_y);
            QPointF p2 = path_points_[i] * vt.scale + QPointF(vt.offset_x, vt.offset_y);
            painter.drawLine(p1, p2);
        }
    }

    // 绘制局部路径（绿色实线，线宽3）
    if (!local_path_points_.empty()) {
        painter.setPen(QPen(QColor(0, 255, 0), 3));
        for (size_t i = 1; i < local_path_points_.size(); ++i) {
            QPointF p1 = local_path_points_[i - 1] * vt.scale + QPointF(vt.offset_x, vt.offset_y);
            QPointF p2 = local_path_points_[i] * vt.scale + QPointF(vt.offset_x, vt.offset_y);
            painter.drawLine(p1, p2);
        }
    }

    // 绘制激光点云（红色小点，点大小2）
    if (!scan_points_.empty()) {
        painter.setPen(QPen(QColor(255, 0, 0), 2));
        for (const auto& pt : scan_points_) {
            QPointF p = pt * vt.scale + QPointF(vt.offset_x, vt.offset_y);
            painter.drawPoint(p);
        }
    }

    if (robot_pose_received_) {
        QPointF center = coord_transformer_.mapToQt(robot_x_, robot_y_);
        center = center * vt.scale + QPointF(vt.offset_x, vt.offset_y);
        // 使用固定像素尺寸显示机器人，避免在高分辨率地图下显示过小
        double pixel_length = 30.0;  // 固定显示尺寸为30像素
        double pixel_width = 24.0;   // 根据长宽比调整

        QTransform transform;
        transform.translate(center.x(), center.y());
        transform.rotate(-robot_yaw_ * 180.0 / M_PI);

        QRectF rect(-pixel_length / 2, -pixel_width / 2, pixel_length, pixel_width);
        QPolygonF robot_rect = transform.map(rect);
        painter.setBrush(QColor(0, 200, 0));
        painter.setPen(Qt::black);
        painter.drawPolygon(robot_rect);

        QPolygonF triangle;
        // 使用固定箭头尺寸，避免在高分辨率地图下显示过小
        double arrow_size = 10.0;  // 固定箭头尺寸为10像素
        triangle << QPointF(pixel_length / 2, 0)
                 << QPointF(pixel_length / 2 - arrow_size, -arrow_size / 2)
                 << QPointF(pixel_length / 2 - arrow_size, arrow_size / 2);
        QPolygonF triangle_transformed = transform.map(triangle);
        painter.drawPolygon(triangle_transformed);
    }

    if (goal_pose_received_) {
        QPointF center = coord_transformer_.mapToQt(goal_x_, goal_y_);
        center = center * vt.scale + QPointF(vt.offset_x, vt.offset_y);
        // 使用固定箭头尺寸，避免在高分辨率地图下显示过小
        double arrow_size = 25.0;  // 固定目标箭头尺寸为25像素

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
        // 使用固定箭头尺寸，与目标箭头保持一致
        double arrow_size = 25.0;  // 固定拖拽预览箭头尺寸为25像素

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

        ViewTransform vt = calculateViewTransform(width(), height(), map_image_.width(), map_image_.height());

        // 将屏幕坐标转换为地图坐标
        QPointF adjusted_pos = (mouse_press_pos_ - QPointF(vt.offset_x, vt.offset_y)) / vt.scale;
        double x, y;
        coord_transformer_.qtToMap(adjusted_pos, x, y);

        double yaw = std::atan2(
            mouse_press_pos_.y() - mouse_current_pos_.y(),
            mouse_current_pos_.x() - mouse_press_pos_.x()
        );

        // 存储目标但不发布
        {
            QWriteLocker locker(&data_lock_);
            goal_x_ = x;
            goal_y_ = y;
            goal_yaw_ = yaw;
            goal_pose_received_ = true;
        }

        // 发射预览信号（不发布到话题）
        Q_EMIT goalPosePreview(x, y, yaw);
        update();
    }
}

void Nav2ViewWidget::publishCurrentGoal()
{
    QReadLocker locker(&data_lock_);
    if (!goal_pose_received_) {
        return;
    }

    // 发布新目标时，清除手动清除标志，允许接收话题更新
    goal_cleared_manually_ = false;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = goal_x_;
    goal_msg.pose.position.y = goal_y_;
    goal_msg.pose.position.z = 0.0;

    goal_msg.pose.orientation = GeometryUtils::yawToQuaternion(goal_yaw_);

    goal_pose_pub_->publish(goal_msg);
}

void Nav2ViewWidget::clearGoal()
{
    QWriteLocker locker(&data_lock_);
    goal_x_ = 0.0;
    goal_y_ = 0.0;
    goal_yaw_ = 0.0;
    goal_pose_received_ = false;
    goal_cleared_manually_ = true;  // 标记已手动清除，忽略后续话题更新
}
