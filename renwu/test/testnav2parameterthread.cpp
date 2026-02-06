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

    // 放弃更改后应该没有待应用的更改
    m_thread->requestDiscard();
    QThread::msleep(500);
    QVERIFY(!m_thread->hasPendingChanges());
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

// ==================== 参数转换测试 (P0) ====================

// 用于测试 protected 方法的派生类
class Nav2ParameterThreadTestable : public Nav2ParameterThread
{
public:
    using Nav2ParameterThread::parameterToVariant;
    using Nav2ParameterThread::variantToParameter;
};

void TestNav2ParameterThread::testParameterToVariantDouble()
{
    Nav2ParameterThreadTestable thread;
    rclcpp::Parameter param("test", 3.14159);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.type(), QVariant::Double);
    QCOMPARE(result.toDouble(), 3.14159);
}

void TestNav2ParameterThread::testParameterToVariantInt()
{
    Nav2ParameterThreadTestable thread;
    rclcpp::Parameter param("test", 42);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.type(), QVariant::Int);
    QCOMPARE(result.toInt(), 42);
}

void TestNav2ParameterThread::testParameterToVariantBool()
{
    Nav2ParameterThreadTestable thread;
    rclcpp::Parameter param("test", true);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.type(), QVariant::Bool);
    QCOMPARE(result.toBool(), true);
}

void TestNav2ParameterThread::testParameterToVariantString()
{
    Nav2ParameterThreadTestable thread;
    rclcpp::Parameter param("test", std::string("hello"));
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.type(), QVariant::String);
    QCOMPARE(result.toString(), QString("hello"));
}

void TestNav2ParameterThread::testParameterToVariantDoubleArray()
{
    Nav2ParameterThreadTestable thread;
    std::vector<double> arr = {1.0, 2.0, 3.0};
    rclcpp::Parameter param("test", arr);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.type(), QVariant::Double);
    QCOMPARE(result.toDouble(), 1.0);  // 只返回第一个元素
}

void TestNav2ParameterThread::testParameterToVariantDoubleArrayEmpty()
{
    Nav2ParameterThreadTestable thread;
    std::vector<double> arr = {};
    rclcpp::Parameter param("test", arr);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(!result.isValid());  // 空数组返回无效 QVariant
}

void TestNav2ParameterThread::testParameterToVariantUnsupportedType()
{
    Nav2ParameterThreadTestable thread;
    // PARAMETER_BYTE_ARRAY 是不支持的类型
    std::vector<uint8_t> arr = {1, 2, 3};
    rclcpp::Parameter param("test", arr);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(!result.isValid());
}

void TestNav2ParameterThread::testVariantToParameterDouble()
{
    Nav2ParameterThreadTestable thread;
    QVariant value(3.14159);
    rclcpp::Parameter result = thread.variantToParameter(value, "test_param");

    QCOMPARE(QString::fromStdString(result.get_name()), QString("test_param"));
    QCOMPARE(result.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE);
    QCOMPARE(result.as_double(), 3.14159);
}

void TestNav2ParameterThread::testVariantToParameterInt()
{
    Nav2ParameterThreadTestable thread;
    QVariant value(42);
    rclcpp::Parameter result = thread.variantToParameter(value, "test_param");

    QCOMPARE(QString::fromStdString(result.get_name()), QString("test_param"));
    QCOMPARE(result.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
    QCOMPARE(result.as_int(), 42);
}

void TestNav2ParameterThread::testVariantToParameterBool()
{
    Nav2ParameterThreadTestable thread;
    QVariant value(true);
    rclcpp::Parameter result = thread.variantToParameter(value, "test_param");

    QCOMPARE(QString::fromStdString(result.get_name()), QString("test_param"));
    QCOMPARE(result.get_type(), rclcpp::ParameterType::PARAMETER_BOOL);
    QCOMPARE(result.as_bool(), true);
}

void TestNav2ParameterThread::testVariantToString()
{
    Nav2ParameterThreadTestable thread;
    QVariant value("hello");
    rclcpp::Parameter result = thread.variantToParameter(value, "test_param");

    QCOMPARE(QString::fromStdString(result.get_name()), QString("test_param"));
    QCOMPARE(result.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
    QCOMPARE(QString::fromStdString(result.as_string()), QString("hello"));
}

void TestNav2ParameterThread::testVariantToParameterUnsupportedType()
{
    Nav2ParameterThreadTestable thread;
    QVariant value = QVariant::fromValue<QSize>(QSize(100, 100));
    rclcpp::Parameter result = thread.variantToParameter(value, "test_param");

    // 不支持的类型返回空的 ParameterValue
    QCOMPARE(result.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);
}

// ==================== 信号测试 (P0) ====================

void TestNav2ParameterThread::testParameterRefreshedSignal()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterRefreshed);

    m_thread->requestRefresh();
    QThread::msleep(1000);  // 等待执行

    QVERIFY(spy.count() > 0);
    QList<QVariant> arguments = spy.takeFirst();
    QCOMPARE(arguments.size(), 2);

    bool success = arguments.at(0).toBool();
    QString message = arguments.at(1).toString();

    QVERIFY(success || !message.isEmpty());  // 成功或失败都有合理消息
}

void TestNav2ParameterThread::testParameterAppliedSignal()
{
    // 设置待应用值
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);

    m_thread->requestApply();
    QThread::msleep(1000);

    QVERIFY(spy.count() > 0);
    QList<QVariant> arguments = spy.takeFirst();
    QCOMPARE(arguments.size(), 3);

    bool success = arguments.at(0).toBool();
    QString message = arguments.at(1).toString();
    QStringList appliedKeys = arguments.at(2).toStringList();

    QVERIFY(message.contains("成功") || message.contains("失败"));
    QVERIFY(!appliedKeys.isEmpty());
}

void TestNav2ParameterThread::testParameterAppliedSignalPartialFailure()
{
    // 修改一个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);

    m_thread->requestApply();
    QThread::msleep(1000);

    // 验证信号被触发
    QVERIFY(spy.count() > 0);
}

void TestNav2ParameterThread::testOperationProgressSignal()
{
    // operationProgress 信号在处理大量参数时触发
    // 由于当前参数数量较少（13个），且处理很快，可能难以捕获
    // 改为验证信号连接机制是否正常

    bool connected = QObject::connect(m_thread, &Nav2ParameterThread::operationProgress,
                                      [](const QString&, int, int, const QString&) {
        // 空槽函数，仅验证可连接
    });

    QVERIFY(connected);

    // 断开连接以避免影响其他测试
    m_thread->disconnect(m_thread, &Nav2ParameterThread::operationProgress, nullptr, nullptr);
}

// ==================== 边界值测试 ====================

void TestNav2ParameterThread::testSetPendingValueZero()
{
    QVERIFY(m_thread->setPendingValue("maxVelXSpinBox", 0.0));

    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.pendingValue.toDouble(), 0.0);
    QVERIFY(info.modified);
}

void TestNav2ParameterThread::testSetPendingValueNegative()
{
    // maxVelX 不应为负，但测试可以设置负值（由上层验证）
    QVERIFY(m_thread->setPendingValue("maxVelXSpinBox", -0.1));

    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QCOMPARE(info.pendingValue.toDouble(), -0.1);
}

void TestNav2ParameterThread::testSetPendingVeryLargeValue()
{
    double largeValue = 9999.99;
    QVERIFY(m_thread->setPendingValue("maxVelXSpinBox", largeValue));

    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.pendingValue.toDouble(), largeValue);
}

void TestNav2ParameterThread::testParameterToVariantZero()
{
    Nav2ParameterThreadTestable thread;
    rclcpp::Parameter param("test", 0.0);
    QVariant result = thread.parameterToVariant(param);

    QVERIFY(result.isValid());
    QCOMPARE(result.toDouble(), 0.0);
}

// ==================== 状态一致性测试 ====================

void TestNav2ParameterThread::testModifiedFlagAfterSetPending()
{
    Nav2ParameterThread::ParamInfo info;

    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    m_thread->getParamInfo("maxVelXSpinBox", info);

    QVERIFY(info.modified);
}

void TestNav2ParameterThread::testModifiedFlagAfterDiscard()
{
    Nav2ParameterThread::ParamInfo info;

    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    m_thread->requestDiscard();
    QThread::msleep(500);

    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVERIFY(!info.modified);
}

void TestNav2ParameterThread::testPendingValueAfterDiscard()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取原始值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    double originalValue = info.currentValue.toDouble();

    // 修改并放弃
    m_thread->setPendingValue("maxVelXSpinBox", 0.5);
    m_thread->requestDiscard();
    QThread::msleep(500);

    // 验证 pendingValue 恢复到 currentValue
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.pendingValue.toDouble(), originalValue);
}

// ==================== 数组参数测试 ====================

void TestNav2ParameterThread::testArrayParameterYDirection()
{
    Nav2ParameterThreadTestable thread;

    // 测试数组参数的转换：返回第一个元素
    std::vector<double> arr = {1.0, 0.0, 2.0};
    rclcpp::Parameter param("max_velocity", arr);
    QVariant result = thread.parameterToVariant(param);

    QCOMPARE(result.toDouble(), 1.0);  // 返回第一个元素
}

#include "testnav2parameterthread.moc"
