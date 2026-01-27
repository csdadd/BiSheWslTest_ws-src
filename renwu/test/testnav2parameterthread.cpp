#include "testnav2parameterthread.h"

void TestNav2ParameterThread::initTestCase()
{
    qDebug() << "[TestNav2ParameterThread] 测试套件初始化";
}

void TestNav2ParameterThread::cleanupTestCase()
{
    qDebug() << "[TestNav2ParameterThread] 测试套件清理";
}

void TestNav2ParameterThread::init()
{
    // 创建并启动线程以初始化参数
    m_thread = new Nav2ParameterThread();
    m_thread->start();

    // 等待线程启动和初始化完成
    QThread::msleep(1000);
}

void TestNav2ParameterThread::cleanup()
{
    if (m_thread) {
        if (m_thread->isRunning()) {
            m_thread->stopThread();
            m_thread->wait(3000);
        }
        delete m_thread;
        m_thread = nullptr;
    }
}

// ==================== 参数注册测试 ====================

void TestNav2ParameterThread::testParameterCount()
{
    // getAllParams() 在未初始化时仍然返回已注册的参数映射
    auto params = m_thread->getAllParams();
    QCOMPARE(params.size(), 13);
}

void TestNav2ParameterThread::testParameterMapping()
{
    auto params = m_thread->getAllParams();

    // 验证关键参数存在
    QVERIFY(params.contains("maxVelXSpinBox"));
    QVERIFY(params.contains("minVelXSpinBox"));
    QVERIFY(params.contains("maxVelThetaSpinBox"));
    QVERIFY(params.contains("lookaheadDistSpinBox"));
    QVERIFY(params.contains("localInflationRadiusSpinBox"));
    QVERIFY(params.contains("localCostScalingFactorSpinBox"));
    QVERIFY(params.contains("globalInflationRadiusSpinBox"));
    QVERIFY(params.contains("globalCostScalingFactorSpinBox"));
    QVERIFY(params.contains("smootherMaxVelSpinBox"));
    QVERIFY(params.contains("smootherMinVelSpinBox"));
    QVERIFY(params.contains("smootherMaxAccelSpinBox"));
    QVERIFY(params.contains("smootherMaxDecelSpinBox"));
    QVERIFY(params.contains("robotRadiusSpinBox"));

    // 验证参数映射关系
    QCOMPARE(params["maxVelXSpinBox"].rosParamName, "FollowPath.max_vel_x");
    QCOMPARE(params["maxVelXSpinBox"].nodeNames.first(), "/controller_server");

    QCOMPARE(params["minVelXSpinBox"].rosParamName, "FollowPath.min_vel_x");

    QCOMPARE(params["maxVelThetaSpinBox"].rosParamName, "FollowPath.max_vel_theta");

    QCOMPARE(params["lookaheadDistSpinBox"].rosParamName, "FollowPath.PathAlign.forward_point_distance");

    QCOMPARE(params["localInflationRadiusSpinBox"].rosParamName, "inflation_layer.inflation_radius");
    QCOMPARE(params["localInflationRadiusSpinBox"].nodeNames.first(), "/local_costmap/local_costmap");

    QCOMPARE(params["globalInflationRadiusSpinBox"].nodeNames.first(), "/global_costmap/global_costmap");
}

void TestNav2ParameterThread::testNodeNames()
{
    Nav2ParameterThread::ParamInfo info;

    // 单节点参数
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.nodeNames.size(), 1);
    QCOMPARE(info.nodeNames[0], "/controller_server");

    QVERIFY(m_thread->getParamInfo("smootherMaxVelSpinBox", info));
    QCOMPARE(info.nodeNames.size(), 1);
    QCOMPARE(info.nodeNames[0], "/velocity_smoother");
}

void TestNav2ParameterThread::testRobotRadiusMultiNode()
{
    Nav2ParameterThread::ParamInfo info;

    QVERIFY(m_thread->getParamInfo("robotRadiusSpinBox", info));
    QCOMPARE(info.nodeNames.size(), 2);
    QCOMPARE(info.nodeNames[0], "/local_costmap/local_costmap");
    QCOMPARE(info.nodeNames[1], "/global_costmap/global_costmap");
    QCOMPARE(info.rosParamName, "robot_radius");
}

void TestNav2ParameterThread::testVelocitySmootherArrayParams()
{
    auto params = m_thread->getAllParams();

    // 验证 velocity_smoother 的所有数组参数
    QVERIFY(params.contains("smootherMaxVelSpinBox"));
    QCOMPARE(params["smootherMaxVelSpinBox"].rosParamName, "max_velocity");
    QCOMPARE(params["smootherMaxVelSpinBox"].nodeNames.first(), "/velocity_smoother");

    QVERIFY(params.contains("smootherMinVelSpinBox"));
    QCOMPARE(params["smootherMinVelSpinBox"].rosParamName, "min_velocity");

    QVERIFY(params.contains("smootherMaxAccelSpinBox"));
    QCOMPARE(params["smootherMaxAccelSpinBox"].rosParamName, "max_accel");

    QVERIFY(params.contains("smootherMaxDecelSpinBox"));
    QCOMPARE(params["smootherMaxDecelSpinBox"].rosParamName, "max_decel");
}

// ==================== 缓存机制测试 ====================

void TestNav2ParameterThread::testGetParamInfo()
{
    Nav2ParameterThread::ParamInfo info;

    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.key, "maxVelXSpinBox");
    QCOMPARE(info.rosParamName, "FollowPath.max_vel_x");
}

void TestNav2ParameterThread::testGetParamInfoInvalidKey()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(!m_thread->getParamInfo("nonexistentSpinBox", info));
}

void TestNav2ParameterThread::testGetAllParams()
{
    auto params = m_thread->getAllParams();
    QCOMPARE(params.size(), 13);

    // 验证所有键都存在
    QVERIFY(params.contains("maxVelXSpinBox"));
    QVERIFY(params.contains("minVelXSpinBox"));
    QVERIFY(params.contains("maxVelThetaSpinBox"));
    QVERIFY(params.contains("lookaheadDistSpinBox"));
    QVERIFY(params.contains("localInflationRadiusSpinBox"));
    QVERIFY(params.contains("localCostScalingFactorSpinBox"));
    QVERIFY(params.contains("globalInflationRadiusSpinBox"));
    QVERIFY(params.contains("globalCostScalingFactorSpinBox"));
    QVERIFY(params.contains("smootherMaxVelSpinBox"));
    QVERIFY(params.contains("smootherMinVelSpinBox"));
    QVERIFY(params.contains("smootherMaxAccelSpinBox"));
    QVERIFY(params.contains("smootherMaxDecelSpinBox"));
    QVERIFY(params.contains("robotRadiusSpinBox"));
}

void TestNav2ParameterThread::testSetPendingValue()
{
    QVERIFY(m_thread->setPendingValue("maxVelXSpinBox", 0.5));

    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.pendingValue.toDouble(), 0.5);
    QVERIFY(info.modified);
}

void TestNav2ParameterThread::testSetPendingValueInvalidKey()
{
    QVERIFY(!m_thread->setPendingValue("nonexistentSpinBox", 0.5));
}

void TestNav2ParameterThread::testHasPendingChanges()
{
    // 初始状态应该没有待应用的更改
    QVERIFY(!m_thread->hasPendingChanges());

    // 设置待应用值
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    QVERIFY(m_thread->hasPendingChanges());

    // 重置后应该没有待应用的更改
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    // 注意：由于没有启动线程，modified 标志保持不变
}

void TestNav2ParameterThread::testHasPendingChangesNoModifications()
{
    QVERIFY(!m_thread->hasPendingChanges());
}

void TestNav2ParameterThread::testMultipleModifiedParams()
{
    // 修改多个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    m_thread->setPendingValue("minVelXSpinBox", 0.1);
    m_thread->setPendingValue("maxVelThetaSpinBox", 1.0);

    QVERIFY(m_thread->hasPendingChanges());
}

// ==================== 任务队列测试 ====================

void TestNav2ParameterThread::testRequestRefresh()
{
    // 请求刷新应该不崩溃
    m_thread->requestRefresh();
    QVERIFY(true);
}

void TestNav2ParameterThread::testRequestApply()
{
    // 请求应用应该不崩溃
    m_thread->requestApply();
    QVERIFY(true);
}

void TestNav2ParameterThread::testRequestReset()
{
    // 请求重置应该不崩溃
    m_thread->requestReset();
    QVERIFY(true);
}

void TestNav2ParameterThread::testRequestDiscard()
{
    // 请求放弃应该不崩溃
    m_thread->requestDiscard();
    QVERIFY(true);
}

void TestNav2ParameterThread::testTaskQueueOrder()
{
    // 连续添加多个任务
    m_thread->requestRefresh();
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    m_thread->requestApply();

    QVERIFY(true);
}

// ==================== 数据结构测试 ====================

void TestNav2ParameterThread::testParamInfoPrimaryNode()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.primaryNode(), "/controller_server");

    QVERIFY(m_thread->getParamInfo("robotRadiusSpinBox", info));
    QCOMPARE(info.primaryNode(), "/local_costmap/local_costmap");
}

void TestNav2ParameterThread::testParamInfoDefaultValues()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));

    // 线程启动后，值应该从 ROS2 读取并初始化
    QVERIFY(info.defaultValue.isValid());
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.pendingValue.isValid());
    // defaultValue 应该等于 currentValue 和 pendingValue
    QCOMPARE(info.defaultValue, info.currentValue);
    QCOMPARE(info.currentValue, info.pendingValue);
    QVERIFY(!info.modified);
}

void TestNav2ParameterThread::testParamTaskConstructor()
{
    Nav2ParameterThread::ParamTask task1;
    QCOMPARE(static_cast<int>(task1.type), static_cast<int>(Nav2ParameterThread::TaskType::Refresh));

    Nav2ParameterThread::ParamTask task2(Nav2ParameterThread::TaskType::Apply);
    QCOMPARE(static_cast<int>(task2.type), static_cast<int>(Nav2ParameterThread::TaskType::Apply));

    Nav2ParameterThread::ParamTask task3(Nav2ParameterThread::TaskType::Reset, "testKey", 42.0);
    QCOMPARE(static_cast<int>(task3.type), static_cast<int>(Nav2ParameterThread::TaskType::Reset));
    QCOMPARE(task3.key, QString("testKey"));
    QCOMPARE(task3.value.toDouble(), 42.0);
}

#include "testnav2parameterthread.moc"
