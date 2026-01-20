#ifndef ROBOT_MODEL_VIEWER_MAIN_WINDOW_HPP_
#define ROBOT_MODEL_VIEWER_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QPointF>
#include <vector>
#include "robot_model_viewer/robot_model_node.hpp"

class DrawingWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget* parent = nullptr);
    virtual ~MainWindow();

private Q_SLOTS:
    void updateDisplay();

private:
    void setupUI();

    RobotModelNode* robot_node_;
    DrawingWidget* drawing_widget_;
    QTimer* update_timer_;
};

class DrawingWidget : public QWidget {
    Q_OBJECT

public:
    explicit DrawingWidget(QWidget* parent = nullptr);

    void setFootprintPoints(const std::vector<QPointF>& points);
    void setRobotPosition(const QPointF& position);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::vector<QPointF> footprint_points_;
    QPointF robot_position_;
};

#endif