#include "robot_model_viewer/main_window.hpp"
#include <QVBoxLayout>
#include <QLabel>

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent), robot_node_(nullptr) {
    robot_node_ = new RobotModelNode(argc, argv);
    if (!robot_node_->init()) {
        return;
    }

    setupUI();

    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplay);
    update_timer_->start(50);

    resize(800, 600);
    setWindowTitle("Robot Model Viewer");
}

MainWindow::~MainWindow() {
    if (robot_node_) {
        delete robot_node_;
    }
}

void MainWindow::setupUI() {
    drawing_widget_ = new DrawingWidget(this);
    setCentralWidget(drawing_widget_);
}

void MainWindow::updateDisplay() {
    drawing_widget_->setFootprintPoints(robot_node_->getFootprintPoints());
    drawing_widget_->setRobotPosition(robot_node_->getRobotPosition());
    drawing_widget_->update();
}

DrawingWidget::DrawingWidget(QWidget* parent)
    : QWidget(parent), robot_position_(0.0, 0.0) {
    setMinimumSize(600, 600);
    setAutoFillBackground(true);
    QPalette palette = this->palette();
    palette.setColor(QPalette::Window, Qt::white);
    setPalette(palette);
}

void DrawingWidget::setFootprintPoints(const std::vector<QPointF>& points) {
    footprint_points_ = points;
}

void DrawingWidget::setRobotPosition(const QPointF& position) {
    robot_position_ = position;
}

void DrawingWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int width = this->width();
    int height = this->height();
    int center_x = width / 2;
    int center_y = height / 2;

    double scale = 50.0;

    painter.fillRect(0, 0, width, height, Qt::white);

    painter.setPen(QPen(Qt::lightGray, 1, Qt::DashLine));
    painter.drawLine(center_x, 0, center_x, height);
    painter.drawLine(0, center_y, width, center_y);

    if (footprint_points_.empty()) {
        painter.setPen(Qt::black);
        painter.drawText(center_x - 100, center_y, "Waiting for footprint data...");
        return;
    }

    std::vector<QPointF> draw_points;
    for (const auto& point : footprint_points_) {
        double relative_x = point.x() - robot_position_.x();
        double relative_y = point.y() - robot_position_.y();

        int screen_x = center_x + static_cast<int>(relative_x * scale);
        int screen_y = center_y - static_cast<int>(relative_y * scale);

        draw_points.emplace_back(screen_x, screen_y);
    }

    painter.setPen(QPen(Qt::blue, 2));
    painter.setBrush(QBrush(QColor(0, 0, 255, 50)));

    if (!draw_points.empty()) {
        QPolygonF polygon;
        for (const auto& point : draw_points) {
            polygon.append(point);
        }
        painter.drawPolygon(polygon);

        painter.setPen(QPen(Qt::red, 3));
        for (size_t i = 0; i < draw_points.size(); ++i) {
            painter.drawEllipse(draw_points[i], 3, 3);
        }
    }

    painter.setPen(Qt::black);
    painter.drawText(10, 20, QString("Robot Position: (%1, %2)")
        .arg(robot_position_.x(), 0, 'f', 2)
        .arg(robot_position_.y(), 0, 'f', 2));
    painter.drawText(10, 40, QString("Footprint Points: %1")
        .arg(footprint_points_.size()));
}