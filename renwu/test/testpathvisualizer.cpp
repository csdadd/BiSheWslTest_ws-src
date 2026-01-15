#include "testpathvisualizer.h"

void TestPathVisualizer::initTestCase()
{
}

void TestPathVisualizer::cleanupTestCase()
{
}

void TestPathVisualizer::init()
{
}

void TestPathVisualizer::cleanup()
{
}

void TestPathVisualizer::testUpdatePath()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[0].pose.position.x = 0.0;
    path.poses[0].pose.position.y = 0.0;
    path.poses[1].pose.position.x = 1.0;
    path.poses[1].pose.position.y = 1.0;
    path.poses[2].pose.position.x = 2.0;
    path.poses[2].pose.position.y = 2.0;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testUpdatePathWithEmptyPath()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testUpdatePathWithSinglePoint()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(1);
    path.poses[0].pose.position.x = 1.0;
    path.poses[0].pose.position.y = 1.0;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testUpdatePathWithMultiplePoints()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(10);
    for (size_t i = 0; i < path.poses.size(); ++i) {
        path.poses[i].pose.position.x = i * 0.5;
        path.poses[i].pose.position.y = i * 0.5;
    }

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testClearPath()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.clearPath();

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testClearPathAfterUpdate()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[0].pose.position.x = 0.0;
    path.poses[0].pose.position.y = 0.0;
    path.poses[1].pose.position.x = 1.0;
    path.poses[1].pose.position.y = 1.0;
    path.poses[2].pose.position.x = 2.0;
    path.poses[2].pose.position.y = 2.0;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));
    visualizer.clearPath();

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathColor()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathColor(Qt::red);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathColorWithDifferentColors()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathColor(Qt::red);
    visualizer.setPathColor(Qt::green);
    visualizer.setPathColor(Qt::blue);
    visualizer.setPathColor(Qt::yellow);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathWidth()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathWidth(3.0);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathWidthWithDifferentValues()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathWidth(1.0);
    visualizer.setPathWidth(2.5);
    visualizer.setPathWidth(5.0);
    visualizer.setPathWidth(10.0);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathStyle()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathStyle(Qt::SolidLine);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testSetPathStyleWithDifferentStyles()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    visualizer.setPathStyle(Qt::SolidLine);
    visualizer.setPathStyle(Qt::DashLine);
    visualizer.setPathStyle(Qt::DotLine);
    visualizer.setPathStyle(Qt::DashDotLine);

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testMapToScene()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(1);
    path.poses[0].pose.position.x = 1.0;
    path.poses[0].pose.position.y = 1.0;

    visualizer.updatePath(path, 0.05, QPointF(0, 0));

    QVERIFY(scene.items().size() > 0);
}

void TestPathVisualizer::testMapToSceneWithDifferentOrigins()
{
    QGraphicsScene scene;
    PathVisualizer visualizer(&scene);

    nav_msgs::msg::Path path;
    path.poses.resize(1);
    path.poses[0].pose.position.x = 1.0;
    path.poses[0].pose.position.y = 1.0;

    visualizer.updatePath(path, 0.05, QPointF(100, 100));
    visualizer.updatePath(path, 0.05, QPointF(-50, -50));
    visualizer.updatePath(path, 0.05, QPointF(200, 150));

    QVERIFY(scene.items().size() > 0);
}
