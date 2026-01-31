#ifndef TESTNAV2VIEWWIDGET_H
#define TESTNAV2VIEWWIDGET_H

#include <QtTest/QtTest>
#include <QImage>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>
#include <QSignalSpy>
#include <memory>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "nav2viewwidget.h"

class TestNav2ViewWidget : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    // 基础功能测试
    void testConstruction();
    void testConstructionWithNullNode();
    void testSetRobotSize();

    // 地图加载测试
    void testLoadValidMap();
    void testLoadInvalidMap();
    void testLoadMapWithMissingImage();

    // 坐标转换测试
    void testMapToQtConversion();
    void testQtToMapConversion();
    void testCoordinateConversionRoundtrip();

    // ROS回调测试（线程安全）
    void testPlanCallbackThreadSafety();
    void testAmclPoseCallbackThreadSafety();
    void testGoalPoseCallbackThreadSafety();
    void testConcurrentCallbacks();

    // 鼠标交互测试
    void testMousePressAndRelease();
    void testMouseDrag();
    void testMouseReleaseSetsGoal();

    // 目标发布测试
    void testPublishCurrentGoal();
    void testPublishGoalWithoutGoal();
    void testClearGoal();

    // 信号测试
    void testDataUpdatedSignal();
    void testGoalPosePreviewSignal();

    // 绘制测试
    void testPaintEventWithNoData();
    void testPaintEventWithPath();
    void testPaintEventWithRobotPose();
    void testPaintEventWithGoalPose();

    // 边界条件测试
    void testEmptyPath();
    void testLargePath();
    void testExtremeCoordinates();
};

#endif
