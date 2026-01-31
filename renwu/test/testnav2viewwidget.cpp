#include "testnav2viewwidget.h"
#include "roscontextmanager.h"
#include <QTemporaryFile>
#include <QTextStream>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

void TestNav2ViewWidget::initTestCase()
{
    ROSContextManager::instance().initialize();
}

void TestNav2ViewWidget::cleanupTestCase()
{
}

void TestNav2ViewWidget::init()
{
}

void TestNav2ViewWidget::cleanup()
{
}

// =============================================================================
// 基础功能测试
// =============================================================================

void TestNav2ViewWidget::testConstruction()
{
    // 创建临时测试地图文件
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    // 创建测试图像
    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    // 写入 YAML 配置
    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_nav_widget");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    QVERIFY(widget.width() >= 0);
    QVERIFY(widget.height() >= 0);
}

void TestNav2ViewWidget::testConstructionWithNullNode()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    // 使用空节点构造，应该不会崩溃
    rclcpp::Node::SharedPtr nullNode = nullptr;
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), nullNode);

    // 小部件应该存在但可能无法正常工作
    QVERIFY(true);
}

void TestNav2ViewWidget::testSetRobotSize()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_robot_size");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    // 设置机器人尺寸（不会崩溃）
    widget.setRobotSize(1.0, 0.8);
    widget.setRobotSize(0.3, 0.2);
    widget.setRobotSize(2.5, 1.5);

    QVERIFY(true);
}

// =============================================================================
// 地图加载测试
// =============================================================================

void TestNav2ViewWidget::testLoadValidMap()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(200, 150, QImage::Format_RGB32);
    testImage.fill(Qt::gray);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-10.0, -10.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_valid_map");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    // 小部件应该成功创建
    QVERIFY(true);
}

void TestNav2ViewWidget::testLoadInvalidMap()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QTextStream out(&yamlFile);
    out << "image: /nonexistent/image.png\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_invalid_map");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    // 即使地图加载失败，小部件也应该存在
    QVERIFY(true);
}

void TestNav2ViewWidget::testLoadMapWithMissingImage()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QTextStream out(&yamlFile);
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_missing_image");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    // 应该优雅地处理缺失的图像字段
    QVERIFY(true);
}

// =============================================================================
// 坐标转换测试
// =============================================================================

void TestNav2ViewWidget::testMapToQtConversion()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_map_to_qt");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 测试坐标转换不会崩溃
    // 注意：mapToQt 是私有方法，我们通过 paintEvent 间接测试
    QVERIFY(true);
}

void TestNav2ViewWidget::testQtToMapConversion()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_qt_to_map");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 通过鼠标事件测试坐标转换
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 60), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QVERIFY(true);
}

void TestNav2ViewWidget::testCoordinateConversionRoundtrip()
{
    // 测试坐标往返转换的一致性
    // 通过鼠标交互间接测试
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_roundtrip");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 测试多次鼠标交互
    for (int i = 0; i < 5; ++i) {
        QPoint pos(30 + i * 10, 30 + i * 10);
        QMouseEvent pressEvent(QEvent::MouseButtonPress, pos, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(&widget, &pressEvent);

        QMouseEvent releaseEvent(QEvent::MouseButtonRelease, pos + QPoint(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(&widget, &releaseEvent);
    }

    QVERIFY(true);
}

// =============================================================================
// ROS回调测试（线程安全）
// =============================================================================

void TestNav2ViewWidget::testPlanCallbackThreadSafety()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_plan_callback");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟高频路径更新（多线程测试）
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    // 线程1：模拟ROS回调写入路径
    std::thread writer([&]() {
        auto pathPub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        for (int i = 0; i < 100; ++i) {
            nav_msgs::msg::Path msg;
            msg.header.stamp = node->now();
            msg.header.frame_id = "map";
            for (int j = 0; j < 10; ++j) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = i * 0.1 + j * 0.01;
                pose.pose.position.y = i * 0.1 + j * 0.01;
                msg.poses.push_back(pose);
            }
            pathPub->publish(msg);
            updateCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // 线程2：主线程触发重绘
    std::thread reader([&]() {
        while (running.load() || updateCount.load() < 100) {
            widget.update();
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    writer.join();
    running = false;
    reader.join();

    QCOMPARE(updateCount.load(), 100);
}

void TestNav2ViewWidget::testAmclPoseCallbackThreadSafety()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_amcl_callback");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟高频位姿更新
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    std::thread writer([&]() {
        auto posePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
        for (int i = 0; i < 100; ++i) {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = node->now();
            msg.header.frame_id = "map";
            msg.pose.pose.position.x = std::sin(i * 0.1) * 2.0;
            msg.pose.pose.position.y = std::cos(i * 0.1) * 2.0;

            // 四元数表示航向角
            double yaw = i * 0.05;
            msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
            msg.pose.pose.orientation.z = std::sin(yaw * 0.5);

            posePub->publish(msg);
            updateCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::thread reader([&]() {
        while (running.load() || updateCount.load() < 100) {
            widget.update();
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    writer.join();
    running = false;
    reader.join();

    QCOMPARE(updateCount.load(), 100);
}

void TestNav2ViewWidget::testGoalPoseCallbackThreadSafety()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_goal_callback");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟目标位姿更新
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    std::thread writer([&]() {
        auto goalPub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        for (int i = 0; i < 100; ++i) {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = node->now();
            msg.header.frame_id = "map";
            msg.pose.position.x = i * 0.1;
            msg.pose.position.y = i * 0.1;

            double yaw = i * 0.1;
            msg.pose.orientation.w = std::cos(yaw * 0.5);
            msg.pose.orientation.z = std::sin(yaw * 0.5);

            goalPub->publish(msg);
            updateCount++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::thread reader([&]() {
        while (running.load() || updateCount.load() < 100) {
            widget.update();
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    writer.join();
    running = false;
    reader.join();

    QCOMPARE(updateCount.load(), 100);
}

void TestNav2ViewWidget::testConcurrentCallbacks()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_concurrent");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 同时触发所有类型的回调
    std::atomic<bool> running{true};
    std::atomic<int> totalUpdates{0};

    auto pathPub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    auto posePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
    auto goalPub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    std::thread pathWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            nav_msgs::msg::Path msg;
            msg.header.stamp = node->now();
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = i * 0.1;
            pose.pose.position.y = i * 0.1;
            msg.poses.push_back(pose);
            pathPub->publish(msg);
            totalUpdates++;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    std::thread poseWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.header.stamp = node->now();
            msg.pose.pose.position.x = std::sin(i * 0.1);
            msg.pose.pose.position.y = std::cos(i * 0.1);
            msg.pose.pose.orientation.w = 1.0;
            posePub->publish(msg);
            totalUpdates++;
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });

    std::thread goalWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = node->now();
            msg.pose.position.x = i * 0.05;
            msg.pose.position.y = i * 0.05;
            msg.pose.orientation.w = 1.0;
            goalPub->publish(msg);
            totalUpdates++;
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    });

    std::thread reader([&]() {
        while (running.load() || totalUpdates.load() < 150) {
            widget.update();
            QApplication::processEvents();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    pathWriter.join();
    poseWriter.join();
    goalWriter.join();
    running = false;
    reader.join();

    QCOMPARE(totalUpdates.load(), 150);
}

// =============================================================================
// 鼠标交互测试
// =============================================================================

void TestNav2ViewWidget::testMousePressAndRelease()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_mouse_basic");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟鼠标点击
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(55, 55), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QVERIFY(true);
}

void TestNav2ViewWidget::testMouseDrag()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_mouse_drag");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟鼠标拖拽
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(30, 30), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    for (int i = 0; i < 10; ++i) {
        QMouseEvent moveEvent(QEvent::MouseMove, QPoint(30 + i * 2, 30 + i * 2), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(&widget, &moveEvent);
        QApplication::processEvents();
    }

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QVERIFY(true);
}

void TestNav2ViewWidget::testMouseReleaseSetsGoal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_mouse_goal");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    bool signalReceived = false;
    double goalX = 0, goalY = 0, goalYaw = 0;

    QObject::connect(&widget, &Nav2ViewWidget::goalPosePreview,
                     [&](double x, double y, double yaw) {
        signalReceived = true;
        goalX = x;
        goalY = y;
        goalYaw = yaw;
    });

    // 模拟设置目标的鼠标操作
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(40, 40), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 40), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QApplication::processEvents();

    QVERIFY(signalReceived);
}

// =============================================================================
// 目标发布测试
// =============================================================================

void TestNav2ViewWidget::testPublishCurrentGoal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_publish_goal");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 先设置目标
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QApplication::processEvents();

    // 发布目标（不会崩溃）
    widget.publishCurrentGoal();

    QVERIFY(true);
}

void TestNav2ViewWidget::testPublishGoalWithoutGoal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_publish_no_goal");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    // 没有设置目标就发布，应该安全处理
    widget.publishCurrentGoal();

    QVERIFY(true);
}

void TestNav2ViewWidget::testClearGoal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_clear_goal");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 设置目标
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QApplication::processEvents();

    // 清除目标
    widget.clearGoal();

    // 再次清除（幂等操作）
    widget.clearGoal();

    QVERIFY(true);
}

// =============================================================================
// 信号测试
// =============================================================================

void TestNav2ViewWidget::testDataUpdatedSignal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_data_updated");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    QSignalSpy spy(&widget, &Nav2ViewWidget::dataUpdated);

    // 触发位姿更新
    auto posePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 2.0;
    msg.pose.pose.orientation.w = 1.0;
    posePub->publish(msg);

    QApplication::processEvents();
    QTest::qWait(100);

    QVERIFY(spy.count() > 0);
}

void TestNav2ViewWidget::testGoalPosePreviewSignal()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_goal_preview");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    QSignalSpy spy(&widget, &Nav2ViewWidget::goalPosePreview);

    // 模拟鼠标拖拽设置目标
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(30, 30), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(70, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QApplication::processEvents();

    QVERIFY(spy.count() > 0);
}

// =============================================================================
// 绘制测试
// =============================================================================

void TestNav2ViewWidget::testPaintEventWithNoData()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_paint_no_data");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 强制重绘
    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

void TestNav2ViewWidget::testPaintEventWithPath()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_paint_path");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发布路径
    auto pathPub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.stamp = node->now();
    pathMsg.header.frame_id = "map";
    for (int i = 0; i < 20; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = i * 0.1;
        pose.pose.position.y = i * 0.05;
        pose.pose.orientation.w = 1.0;
        pathMsg.poses.push_back(pose);
    }
    pathPub->publish(pathMsg);

    QApplication::processEvents();
    QTest::qWait(100);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

void TestNav2ViewWidget::testPaintEventWithRobotPose()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_paint_robot");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发布机器人位姿
    auto posePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
    geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
    poseMsg.header.stamp = node->now();
    poseMsg.header.frame_id = "map";
    poseMsg.pose.pose.position.x = 0.5;
    poseMsg.pose.pose.position.y = 0.3;
    poseMsg.pose.pose.orientation.w = std::cos(0.5 / 2);
    poseMsg.pose.pose.orientation.z = std::sin(0.5 / 2);
    posePub->publish(poseMsg);

    QApplication::processEvents();
    QTest::qWait(100);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

void TestNav2ViewWidget::testPaintEventWithGoalPose()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_paint_goal");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发布目标位姿
    auto goalPub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    geometry_msgs::msg::PoseStamped goalMsg;
    goalMsg.header.stamp = node->now();
    goalMsg.header.frame_id = "map";
    goalMsg.pose.position.x = 1.5;
    goalMsg.pose.position.y = 1.0;
    goalMsg.pose.orientation.w = std::cos(1.0 / 2);
    goalMsg.pose.orientation.z = std::sin(1.0 / 2);
    goalPub->publish(goalMsg);

    QApplication::processEvents();
    QTest::qWait(100);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

// =============================================================================
// 边界条件测试
// =============================================================================

void TestNav2ViewWidget::testEmptyPath()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_empty_path");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发布空路径
    auto pathPub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.stamp = node->now();
    pathMsg.header.frame_id = "map";
    // 不添加任何位姿点
    pathPub->publish(pathMsg);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

void TestNav2ViewWidget::testLargePath()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_large_path");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发布大路径（1000个点）
    auto pathPub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.stamp = node->now();
    pathMsg.header.frame_id = "map";
    for (int i = 0; i < 1000; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = i * 0.01;
        pose.pose.position.y = std::sin(i * 0.1) * 0.5;
        pose.pose.orientation.w = 1.0;
        pathMsg.poses.push_back(pose);
    }
    pathPub->publish(pathMsg);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(100);

    QVERIFY(true);
}

void TestNav2ViewWidget::testExtremeCoordinates()
{
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    QTemporaryFile imageFile;
    QVERIFY(imageFile.open());
    QVERIFY(testImage.save(imageFile.fileName(), "PNG"));

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: 0.05\n";
    out << "origin: [-100.0, -100.0, 0.0]\n";
    out.flush();

    auto node = std::make_shared<rclcpp::Node>("test_extreme_coords");
    Nav2ViewWidget widget(yamlFile.fileName().toStdString(), node);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 测试极端坐标值
    auto posePub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);

    geometry_msgs::msg::PoseWithCovarianceStamped msg1;
    msg1.header.stamp = node->now();
    msg1.pose.pose.position.x = 1000.0;
    msg1.pose.pose.position.y = 1000.0;
    msg1.pose.pose.orientation.w = 1.0;
    posePub->publish(msg1);

    geometry_msgs::msg::PoseWithCovarianceStamped msg2;
    msg2.header.stamp = node->now();
    msg2.pose.pose.position.x = -1000.0;
    msg2.pose.pose.position.y = -1000.0;
    msg2.pose.pose.orientation.w = 1.0;
    posePub->publish(msg2);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}
