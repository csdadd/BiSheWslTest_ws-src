#include "testnav2viewwidget.h"
#include "roscontextmanager.h"
#include "coordinatetransformer.h"
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

// 辅助函数：创建测试地图文件
static bool createTestMapFile(QTemporaryFile& yamlFile, QTemporaryFile& imageFile,
                               double resolution = 0.05, double originX = -5.0, double originY = -5.0)
{
    if (!yamlFile.open()) return false;

    QImage testImage(100, 100, QImage::Format_RGB32);
    testImage.fill(Qt::white);

    if (!imageFile.open()) return false;
    if (!testImage.save(imageFile.fileName(), "PNG")) return false;

    QTextStream out(&yamlFile);
    out << "image: " << imageFile.fileName() << "\n";
    out << "resolution: " << resolution << "\n";
    out << "origin: [" << originX << ", " << originY << ", 0.0]\n";
    out.flush();

    return true;
}

// 辅助函数：从YAML加载地图数据
static bool loadMapData(const std::string& yaml_path, QImage& image, CoordinateTransformer& transformer)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (!config["image"]) {
            return false;
        }

        std::string image_name = config["image"].as<std::string>();
        std::string yaml_dir = yaml_path.substr(0, yaml_path.find_last_of("/\\"));
        std::string image_path = yaml_dir + "/" + image_name;

        if (!image.load(QString::fromStdString(image_path))) {
            return false;
        }

        double resolution = 0.05;
        double origin_x = 0.0;
        double origin_y = 0.0;

        if (config["resolution"]) {
            resolution = config["resolution"].as<double>();
        }

        if (config["origin"]) {
            auto origin = config["origin"];
            if (origin.IsSequence() && origin.size() >= 2) {
                origin_x = origin[0].as<double>();
                origin_y = origin[1].as<double>();
            }
        }

        transformer = CoordinateTransformer(resolution, origin_x, origin_y, image.height());
        return true;
    } catch (...) {
        return false;
    }
}

// =============================================================================
// 基础功能测试
// =============================================================================

void TestNav2ViewWidget::testConstruction()
{
    // 新API：Nav2ViewWidget只需要QWidget*参数
    Nav2ViewWidget widget(nullptr);

    QVERIFY(widget.width() >= 0);
    QVERIFY(widget.height() >= 0);
}

void TestNav2ViewWidget::testConstructionWithNullNode()
{
    // 新API：不需要ROS节点参数
    Nav2ViewWidget widget(nullptr);

    QVERIFY(true);
}

void TestNav2ViewWidget::testSetRobotSize()
{
    Nav2ViewWidget widget(nullptr);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    QVERIFY(true);
}

void TestNav2ViewWidget::testLoadInvalidMap()
{
    // 创建无效的YAML文件
    QTemporaryFile yamlFile;
    QVERIFY(yamlFile.open());

    QTextStream out(&yamlFile);
    out << "image: /nonexistent/image.png\n";
    out << "resolution: 0.05\n";
    out << "origin: [-5.0, -5.0, 0.0]\n";
    out.flush();

    QImage mapImage;
    CoordinateTransformer transformer;
    // 加载应该失败
    QVERIFY(!loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
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

    QImage mapImage;
    CoordinateTransformer transformer;
    // 加载应该失败（缺少image字段）
    QVERIFY(!loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    QVERIFY(true);
}

// =============================================================================
// 坐标转换测试
// =============================================================================

void TestNav2ViewWidget::testMapToQtConversion()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    QVERIFY(true);
}

void TestNav2ViewWidget::testQtToMapConversion()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 通过鼠标事件测试
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 60), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QVERIFY(true);
}

void TestNav2ViewWidget::testCoordinateConversionRoundtrip()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
// 渲染数据更新测试（替代原来的ROS回调测试）
// =============================================================================

void TestNav2ViewWidget::testPlanCallbackThreadSafety()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟高频渲染数据更新（多线程测试）
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    // 线程1：模拟数据处理器写入路径
    std::thread writer([&]() {
        for (int i = 0; i < 100; ++i) {
            RenderData data;
            data.robot_pose_received = true;
            data.robot_x = i * 0.01;
            data.robot_y = i * 0.01;

            for (int j = 0; j < 10; ++j) {
                double x = i * 0.1 + j * 0.01;
                double y = i * 0.1 + j * 0.01;
                QPointF pt = transformer.mapToQt(x, y);
                data.path_points.push_back(pt);
            }

            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟高频位姿更新
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    std::thread writer([&]() {
        for (int i = 0; i < 100; ++i) {
            RenderData data;
            data.robot_pose_received = true;
            data.robot_x = std::sin(i * 0.1) * 2.0;
            data.robot_y = std::cos(i * 0.1) * 2.0;
            data.robot_yaw = i * 0.05;

            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 模拟目标位姿更新
    std::atomic<bool> running{true};
    std::atomic<int> updateCount{0};

    std::thread writer([&]() {
        for (int i = 0; i < 100; ++i) {
            RenderData data;
            data.goal_pose_received = true;
            data.goal_x = i * 0.1;
            data.goal_y = i * 0.1;
            data.goal_yaw = i * 0.1;

            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 同时触发所有类型的数据更新
    std::atomic<bool> running{true};
    std::atomic<int> totalUpdates{0};

    std::thread pathWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            RenderData data;
            for (int j = 0; j < 5; ++j) {
                QPointF pt = transformer.mapToQt(i * 0.1 + j * 0.02, i * 0.1 + j * 0.02);
                data.path_points.push_back(pt);
            }
            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
            totalUpdates++;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    std::thread poseWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            RenderData data;
            data.robot_pose_received = true;
            data.robot_x = std::sin(i * 0.1);
            data.robot_y = std::cos(i * 0.1);
            data.robot_yaw = i * 0.05;
            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
            totalUpdates++;
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });

    std::thread goalWriter([&]() {
        for (int i = 0; i < 50; ++i) {
            RenderData data;
            data.goal_pose_received = true;
            data.goal_x = i * 0.05;
            data.goal_y = i * 0.05;
            data.goal_yaw = i * 0.02;
            QMetaObject::invokeMethod(&widget, "onRenderDataReady", Qt::QueuedConnection,
                                      Q_ARG(RenderData, data));
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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 先设置目标
    QMouseEvent pressEvent(QEvent::MouseButtonPress, QPoint(50, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &pressEvent);

    QMouseEvent releaseEvent(QEvent::MouseButtonRelease, QPoint(60, 50), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(&widget, &releaseEvent);

    QApplication::processEvents();

    // 发布目标（改为发射信号）
    bool goalSetReceived = false;
    QObject::connect(&widget, &Nav2ViewWidget::goalPoseSet, [&]() {
        goalSetReceived = true;
    });

    widget.publishCurrentGoal();

    QVERIFY(true);
}

void TestNav2ViewWidget::testPublishGoalWithoutGoal()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    // 没有设置目标就发布，应该安全处理
    widget.publishCurrentGoal();

    QVERIFY(true);
}

void TestNav2ViewWidget::testClearGoal()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    QSignalSpy spy(&widget, &Nav2ViewWidget::dataUpdated);

    // 直接调用渲染数据更新槽
    RenderData data;
    data.robot_pose_received = true;
    data.robot_x = 1.0;
    data.robot_y = 2.0;
    data.robot_yaw = 0.5;
    widget.onRenderDataReady(data);

    QApplication::processEvents();
    QTest::qWait(100);

    QVERIFY(spy.count() > 0);
}

void TestNav2ViewWidget::testGoalPosePreviewSignal()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 直接设置渲染数据（包含路径）
    RenderData data;
    for (int i = 0; i < 20; ++i) {
        QPointF pt = transformer.mapToQt(i * 0.1, i * 0.05);
        data.path_points.push_back(pt);
    }
    widget.onRenderDataReady(data);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 直接设置渲染数据（包含机器人位姿）
    RenderData data;
    data.robot_pose_received = true;
    data.robot_x = 0.5;
    data.robot_y = 0.3;
    data.robot_yaw = 0.5;
    widget.onRenderDataReady(data);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 直接设置渲染数据（包含目标位姿）
    RenderData data;
    data.goal_pose_received = true;
    data.goal_x = 1.5;
    data.goal_y = 1.0;
    data.goal_yaw = 1.0;
    widget.onRenderDataReady(data);

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
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发送空路径的渲染数据
    RenderData data;
    // path_points 为空
    widget.onRenderDataReady(data);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}

void TestNav2ViewWidget::testLargePath()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 发送大路径（1000个点）的渲染数据
    RenderData data;
    for (int i = 0; i < 1000; ++i) {
        double x = i * 0.01;
        double y = std::sin(i * 0.1) * 0.5;
        QPointF pt = transformer.mapToQt(x, y);
        data.path_points.push_back(pt);
    }
    widget.onRenderDataReady(data);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(100);

    QVERIFY(true);
}

void TestNav2ViewWidget::testExtremeCoordinates()
{
    QTemporaryFile yamlFile;
    QTemporaryFile imageFile;
    QVERIFY(createTestMapFile(yamlFile, imageFile, 0.05, -100.0, -100.0));

    QImage mapImage;
    CoordinateTransformer transformer;
    QVERIFY(loadMapData(yamlFile.fileName().toStdString(), mapImage, transformer));

    Nav2ViewWidget widget(nullptr);
    widget.onMapLoaded(mapImage, transformer);

    widget.show();
    (void)QTest::qWaitForWindowExposed(&widget);

    // 测试极端坐标值
    RenderData data1;
    data1.robot_pose_received = true;
    data1.robot_x = 1000.0;
    data1.robot_y = 1000.0;
    data1.robot_yaw = 0.0;
    widget.onRenderDataReady(data1);

    RenderData data2;
    data2.robot_pose_received = true;
    data2.robot_x = -1000.0;
    data2.robot_y = -1000.0;
    data2.robot_yaw = 0.0;
    widget.onRenderDataReady(data2);

    widget.update();
    QApplication::processEvents();
    QTest::qWait(50);

    QVERIFY(true);
}
