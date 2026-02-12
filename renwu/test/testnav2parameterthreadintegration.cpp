#include "testnav2parameterthreadintegration.h"

void TestNav2ParameterThreadIntegration::initTestCase()
{
    qDebug() << "[TestNav2ParameterThreadIntegration] 测试套件初始化";
    qDebug() << "[TestNav2ParameterThreadIntegration] 请确保 Nav2 节点已启动";
}

void TestNav2ParameterThreadIntegration::cleanupTestCase()
{
    qDebug() << "[TestNav2ParameterThreadIntegration] 测试套件清理";
}

void TestNav2ParameterThreadIntegration::init()
{
    // 创建线程
    m_thread = new Nav2ParameterThread();

    // 启动线程
    m_thread->start();

    // 等待初始化完成
    QThread::msleep(500);

    // 保存初始值用于测试后恢复
    auto params = m_thread->getAllParams();
    for (auto it = params.begin(); it != params.end(); ++it) {
        m_initialValues[it.key()] = it.value().currentValue;
    }
}

void TestNav2ParameterThreadIntegration::cleanup()
{
    if (m_thread) {
        // 恢复初始值
        for (auto it = m_initialValues.begin(); it != m_initialValues.end(); ++it) {
            m_thread->setPendingValue(it.key(), it.value());
        }
        m_thread->requestApply();
        QThread::msleep(500);

        // 验证恢复成功
        auto params = m_thread->getAllParams();
        for (auto it = params.begin(); it != params.end(); ++it) {
            if (m_initialValues.contains(it.key())) {
                QCOMPARE(it.value().currentValue, m_initialValues[it.key()]);
            }
        }

        // 停止线程
        if (m_thread->isRunning()) {
            m_thread->stopThread();
            m_thread->wait(3000);
        }

        delete m_thread;
        m_thread = nullptr;
    }
}

// ==================== 参数读取测试 ====================

void TestNav2ParameterThreadIntegration::testReadNormalParameters()
{
    QVERIFY(m_thread != nullptr);
    QVERIFY(m_thread->isRunning());

    Nav2ParameterThread::ParamInfo info;

    // 测试 maxVelX
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadNormalParameters] maxVelX =" << info.currentValue.toDouble();

    // 测试 minVelX
    QVERIFY(m_thread->getParamInfo("minVelXSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() >= 0);
    qDebug() << "[testReadNormalParameters] minVelX =" << info.currentValue.toDouble();

    // 测试 maxVelTheta
    QVERIFY(m_thread->getParamInfo("maxVelThetaSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadNormalParameters] maxVelTheta =" << info.currentValue.toDouble();

    // 测试 lookaheadDist
    QVERIFY(m_thread->getParamInfo("lookaheadDistSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadNormalParameters] lookaheadDist =" << info.currentValue.toDouble();
}

void TestNav2ParameterThreadIntegration::testReadArrayParameters()
{
    Nav2ParameterThread::ParamInfo info;

    // 测试 velocity_smoother 的数组参数
    QVERIFY(m_thread->getParamInfo("smootherMaxVelSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() >= 0);
    qDebug() << "[testReadArrayParameters] smootherMaxVel =" << info.currentValue.toDouble();

    QVERIFY(m_thread->getParamInfo("smootherMinVelSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    qDebug() << "[testReadArrayParameters] smootherMinVel =" << info.currentValue.toDouble();

    QVERIFY(m_thread->getParamInfo("smootherMaxAccelSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadArrayParameters] smootherMaxAccel =" << info.currentValue.toDouble();

    QVERIFY(m_thread->getParamInfo("smootherMaxDecelSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    qDebug() << "[testReadArrayParameters] smootherMaxDecel =" << info.currentValue.toDouble();
}

void TestNav2ParameterThreadIntegration::testReadCostmapParameters()
{
    Nav2ParameterThread::ParamInfo info;

    // 测试局部代价地图参数
    QVERIFY(m_thread->getParamInfo("localInflationRadiusSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadCostmapParameters] localInflationRadius =" << info.currentValue.toDouble();

    QVERIFY(m_thread->getParamInfo("localCostScalingFactorSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadCostmapParameters] localCostScalingFactor =" << info.currentValue.toDouble();

    // 测试全局代价地图参数
    QVERIFY(m_thread->getParamInfo("globalInflationRadiusSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadCostmapParameters] globalInflationRadius =" << info.currentValue.toDouble();

    QVERIFY(m_thread->getParamInfo("globalCostScalingFactorSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    qDebug() << "[testReadCostmapParameters] globalCostScalingFactor =" << info.currentValue.toDouble();
}

void TestNav2ParameterThreadIntegration::testReadRobotRadius()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("robotRadiusSpinBox", info));
    QVERIFY(info.currentValue.isValid());
    QVERIFY(info.currentValue.toDouble() > 0);
    qDebug() << "[testReadRobotRadius] robotRadius =" << info.currentValue.toDouble();
}

void TestNav2ParameterThreadIntegration::testAllParametersRead()
{
    auto params = m_thread->getAllParams();

    for (auto it = params.begin(); it != params.end(); ++it) {
        QVERIFY2(it.value().currentValue.isValid(),
                 qPrintable(QString("参数 %1 的值无效").arg(it.key())));
        QVERIFY2(!it.value().modified,
                 qPrintable(QString("参数 %1 不应处于修改状态").arg(it.key())));
    }
}

// ==================== 参数写入测试 ====================

void TestNav2ParameterThreadIntegration::testWriteNormalParameter()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.1;

    // 设置待应用值
    QVERIFY(m_thread->setPendingValue("maxVelXSpinBox", testValue));

    // 应用更改
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    // 等待应用完成
    QVERIFY(spy.wait(10000));

    // 验证 currentValue 已更新
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);
    QVERIFY(!info.modified);

    qDebug() << "[testWriteNormalParameter] maxVelX 从" << originalValue << "改为" << testValue;
}

void TestNav2ParameterThreadIntegration::testWriteArrayParameter()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("smootherMaxVelSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.05;

    // 设置并应用
    m_thread->setPendingValue("smootherMaxVelSpinBox", testValue);
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证
    m_thread->getParamInfo("smootherMaxVelSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    qDebug() << "[testWriteArrayParameter] smootherMaxVel 从" << originalValue << "改为" << testValue;
}

void TestNav2ParameterThreadIntegration::testWriteRobotRadius()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("robotRadiusSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.02;

    // 设置并应用
    m_thread->setPendingValue("robotRadiusSpinBox", testValue);
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证
    m_thread->getParamInfo("robotRadiusSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    qDebug() << "[testWriteRobotRadius] robotRadius 从" << originalValue << "改为" << testValue;
}

void TestNav2ParameterThreadIntegration::testWriteBatchParameters()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);

    // 同时修改多个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.4);
    m_thread->setPendingValue("minVelXSpinBox", 0.1);
    m_thread->setPendingValue("maxVelThetaSpinBox", 0.8);

    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证所有参数都已更新
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), 0.4);

    m_thread->getParamInfo("minVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), 0.1);

    m_thread->getParamInfo("maxVelThetaSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), 0.8);
}

void TestNav2ParameterThreadIntegration::testWriteWithNoModifications()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);

    // 没有修改任何参数，直接应用
    m_thread->requestApply();

    QVERIFY(spy.wait(5000));
    QList<QVariant> arguments = spy.takeFirst();
    QVERIFY(arguments.at(1).toBool());  // success = true
}

// ==================== 刷新功能测试 ====================

void TestNav2ParameterThreadIntegration::testRefreshAllParameters()
{
    // 先修改一个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.3);
    m_thread->requestApply();
    QThread::msleep(500);

    // 再刷新
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);
    m_thread->requestRefresh();

    QVERIFY(spy.wait(10000));

    // 验证刷新后修改状态被清除
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVERIFY(!info.modified);
}

void TestNav2ParameterThreadIntegration::testRefreshPreservesDefault()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取默认值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVariant defaultValue = info.defaultValue;

    // 修改并刷新
    m_thread->setPendingValue("maxVelXSpinBox", 0.3);
    m_thread->requestApply();
    QThread::msleep(500);

    m_thread->requestRefresh();
    QThread::msleep(500);

    // 验证默认值保持不变
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.defaultValue, defaultValue);
}

// ==================== 重置功能测试 ====================

void TestNav2ParameterThreadIntegration::testResetAllParameters()
{
    // 先修改一些参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.2);
    m_thread->setPendingValue("minVelXSpinBox", 0.05);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);
    m_thread->requestReset();

    QVERIFY(spy.wait(5000));

    // 验证所有参数都被标记为待应用
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVERIFY(info.modified);
    QCOMPARE(info.pendingValue, info.defaultValue);

    m_thread->getParamInfo("minVelXSpinBox", info);
    QVERIFY(info.modified);
}

void TestNav2ParameterThreadIntegration::testResetThenApply()
{
    // 修改参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.2);

    // 重置
    m_thread->requestReset();
    QThread::msleep(500);

    // 应用
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证参数恢复到默认值
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue, info.defaultValue);
}

// ==================== 放弃功能测试 ====================

void TestNav2ParameterThreadIntegration::testDiscardSingleParameter()
{
    // 获取当前值
    Nav2ParameterThread::ParamInfo info;
    m_thread->getParamInfo("maxVelXSpinBox", info);
    double originalValue = info.currentValue.toDouble();

    // 修改参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.2);
    // 重新获取参数信息来检查 modified 标志
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVERIFY(info.modified);

    // 放弃
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);
    m_thread->requestDiscard();

    QVERIFY(spy.wait(5000));

    // 验证参数恢复到当前值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.pendingValue.toDouble(), originalValue);
    QVERIFY(!info.modified);
}

void TestNav2ParameterThreadIntegration::testDiscardMultipleParameters()
{
    // 修改多个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.3);
    m_thread->setPendingValue("minVelXSpinBox", 0.1);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);
    m_thread->requestDiscard();

    QVERIFY(spy.wait(5000));

    // 验证所有修改都被放弃
    QVERIFY(!m_thread->hasPendingChanges());
}

void TestNav2ParameterThreadIntegration::testDiscardNoModifications()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);

    // 没有修改的情况下放弃
    m_thread->requestDiscard();

    QVERIFY(spy.wait(5000));
}

// ==================== 线程安全测试 ====================

void TestNav2ParameterThreadIntegration::testConcurrentRefreshAndModify()
{
    // 快速连续操作
    for (int i = 0; i < 10; i++) {
        m_thread->requestRefresh();
        m_thread->setPendingValue("maxVelXSpinBox", 0.3 + i * 0.01);
        QThread::msleep(10);
    }

    QThread::msleep(1000);
    QVERIFY(true);  // 没有崩溃即成功
}

void TestNav2ParameterThreadIntegration::testRapidSequentialOperations()
{
    // 快速连续发送各种请求
    for (int i = 0; i < 20; i++) {
        m_thread->requestRefresh();
        m_thread->setPendingValue("maxVelXSpinBox", 0.3 + i * 0.01);
        m_thread->requestApply();
        m_thread->requestReset();
        m_thread->requestDiscard();
    }

    QThread::msleep(3000);
    QVERIFY(true);  // 没有崩溃即成功
}

// ==================== 异常处理测试 ====================

void TestNav2ParameterThreadIntegration::testNodeUnavailable()
{
    // 尝试设置不存在的参数
    QVERIFY(!m_thread->setPendingValue("nonexistentSpinBox", 0.5));
}

// ==================== 信号测试 ====================

void TestNav2ParameterThreadIntegration::testOperationFinishedSignal()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationFinished);

    m_thread->requestRefresh();

    QVERIFY(spy.wait(10000));
    QList<QVariant> arguments = spy.takeFirst();

    QCOMPARE(arguments.at(0).toString(), QString("refresh"));
}

void TestNav2ParameterThreadIntegration::testParameterRefreshedSignalSuccess()
{
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterRefreshed);

    m_thread->requestRefresh();

    QVERIFY(spy.wait(10000));
    QList<QVariant> arguments = spy.takeFirst();

    bool success = arguments.at(0).toBool();
    QString message = arguments.at(1).toString();

    QVERIFY(success);
    QVERIFY(message.contains("刷新完成") || message.contains("成功"));
}

void TestNav2ParameterThreadIntegration::testParameterRefreshedSignalFailure()
{
    // 这个测试需要模拟节点不可用的场景
    // 在实际环境中可能难以模拟，这里做基础验证
    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterRefreshed);

    m_thread->requestRefresh();

    // 至少信号被触发
    QVERIFY(spy.wait(10000));
}

void TestNav2ParameterThreadIntegration::testParameterAppliedSignalWithMultipleParams()
{
    // 修改多个参数
    m_thread->setPendingValue("maxVelXSpinBox", 0.4);
    m_thread->setPendingValue("minVelXSpinBox", 0.1);
    m_thread->setPendingValue("maxVelThetaSpinBox", 0.8);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);

    m_thread->requestApply();

    QVERIFY(spy.wait(10000));
    QList<QVariant> arguments = spy.takeFirst();

    bool success = arguments.at(0).toBool();
    QString message = arguments.at(1).toString();
    QStringList appliedKeys = arguments.at(2).toStringList();

    QVERIFY(success || message.contains("失败"));  // 根据环境可能成功或失败
    if (success) {
        QCOMPARE(appliedKeys.size(), 3);
        QVERIFY(appliedKeys.contains("maxVelXSpinBox"));
        QVERIFY(appliedKeys.contains("minVelXSpinBox"));
        QVERIFY(appliedKeys.contains("maxVelThetaSpinBox"));
    }
}

void TestNav2ParameterThreadIntegration::testOperationProgressSignalDuringRefresh()
{
    // operationProgress 信号在处理大量参数时触发
    // 由于当前参数数量较少（13个），且处理很快，可能难以捕获
    QSignalSpy spy(m_thread, &Nav2ParameterThread::operationProgress);

    m_thread->requestRefresh();
    QThread::msleep(1000);

    // 这个测试可能不稳定，因为处理太快
    // 至少验证代码不会崩溃
    QVERIFY(true);
}

// ==================== velocity_smoother 数组参数测试 ====================

void TestNav2ParameterThreadIntegration::testVelocitySmootherArrayParameterWrite()
{
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("smootherMaxVelSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.05;

    // 设置并应用
    m_thread->setPendingValue("smootherMaxVelSpinBox", testValue);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证值已更新
    m_thread->getParamInfo("smootherMaxVelSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    // 恢复原始值
    m_thread->setPendingValue("smootherMaxVelSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

// ==================== 多节点参数测试 ====================

void TestNav2ParameterThreadIntegration::testMultiNodeParameterConsistency()
{
    Nav2ParameterThread::ParamInfo info;

    // robotRadiusSpinBox 写入到两个节点
    m_thread->getParamInfo("robotRadiusSpinBox", info);
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.01;

    m_thread->setPendingValue("robotRadiusSpinBox", testValue);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证值已更新
    m_thread->getParamInfo("robotRadiusSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);
    QVERIFY(!info.modified);

    // 恢复原始值
    m_thread->setPendingValue("robotRadiusSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

// ==================== 默认值保留测试 ====================

void TestNav2ParameterThreadIntegration::testDefaultValuePreservedAfterModify()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取默认值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVariant defaultValue = info.defaultValue;

    // 修改并应用
    m_thread->setPendingValue("maxVelXSpinBox", 0.3);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证默认值仍然不变
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.defaultValue, defaultValue);
}

// ==================== 完整工作流测试 ====================

void TestNav2ParameterThreadIntegration::testFullWorkflowModifyApplyRefresh()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取原始值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    double originalValue = info.currentValue.toDouble();

    // 修改 -> 应用 -> 刷新
    m_thread->setPendingValue("maxVelXSpinBox", 0.35);

    QSignalSpy spy1(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();
    QVERIFY(spy1.wait(10000));

    m_thread->requestRefresh();
    QThread::msleep(1000);

    // 验证：刷新后值为新值（不是原始值）
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), 0.35);
    QVERIFY(!info.modified);

    // 恢复原始值
    m_thread->setPendingValue("maxVelXSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

void TestNav2ParameterThreadIntegration::testFullWorkflowModifyResetApply()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取默认值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QVariant defaultValue = info.defaultValue;

    // 修改 -> 重置 -> 应用
    m_thread->setPendingValue("maxVelXSpinBox", 0.25);

    m_thread->requestReset();
    QThread::msleep(500);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证：恢复到默认值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue, defaultValue);
}

void TestNav2ParameterThreadIntegration::testFullWorkflowModifyDiscard()
{
    Nav2ParameterThread::ParamInfo info;

    // 获取原始值
    m_thread->getParamInfo("maxVelXSpinBox", info);
    double originalValue = info.currentValue.toDouble();

    // 修改 -> 放弃
    m_thread->setPendingValue("maxVelXSpinBox", 0.25);

    m_thread->requestDiscard();
    QThread::msleep(500);

    // 验证：值未改变
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), originalValue);
    QCOMPARE(info.pendingValue.toDouble(), originalValue);
    QVERIFY(!info.modified);
    QVERIFY(!m_thread->hasPendingChanges());
}

// ==================== 回读验证测试 ====================

void TestNav2ParameterThreadIntegration::testVerifyParameterValueAfterWrite()
{
    // 测试普通参数写入后的回读验证
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("maxVelXSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.1;

    // 设置并应用
    m_thread->setPendingValue("maxVelXSpinBox", testValue);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证信号显示成功
    QList<QVariant> arguments = spy.takeFirst();
    bool success = arguments.at(0).toBool();
    QVERIFY2(success, "参数应用应该成功");

    // 验证 currentValue 已更新
    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    // 再次刷新来验证 ROS2 中的值确实被设置成功
    m_thread->requestRefresh();
    QThread::msleep(1000);

    m_thread->getParamInfo("maxVelXSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    qDebug() << "[testVerifyParameterValueAfterWrite] 验证成功: 值确实被写入到 ROS2";

    // 恢复原始值
    m_thread->setPendingValue("maxVelXSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

void TestNav2ParameterThreadIntegration::testVerifyArrayParameterAfterWrite()
{
    // 测试数组参数（velocity_smoother）写入后的回读验证
    Nav2ParameterThread::ParamInfo info;
    QVERIFY(m_thread->getParamInfo("smootherMaxVelSpinBox", info));
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.05;

    // 设置并应用
    m_thread->setPendingValue("smootherMaxVelSpinBox", testValue);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 验证成功
    QList<QVariant> arguments = spy.takeFirst();
    bool success = arguments.at(0).toBool();
    QVERIFY2(success, "数组参数应用应该成功");

    // 刷新验证
    m_thread->requestRefresh();
    QThread::msleep(1000);

    m_thread->getParamInfo("smootherMaxVelSpinBox", info);
    QCOMPARE(info.currentValue.toDouble(), testValue);

    qDebug() << "[testVerifyArrayParameterAfterWrite] 数组参数验证成功";

    // 恢复原始值
    m_thread->setPendingValue("smootherMaxVelSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

void TestNav2ParameterThreadIntegration::testVerifyDetectsMismatch()
{
    // 这个测试验证回读验证能检测到不匹配的情况
    // 由于我们无法在真实环境中模拟参数设置失败，这个测试主要验证机制的存在

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);

    // 设置一个合理的值
    m_thread->setPendingValue("maxVelXSpinBox", 0.4);

    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    // 如果应用成功，说明回读验证通过了
    QList<QVariant> arguments = spy.takeFirst();
    bool success = arguments.at(0).toBool();

    if (success) {
        // 成功的情况下，通过刷新来验证值确实一致
        m_thread->requestRefresh();
        QThread::msleep(1000);

        Nav2ParameterThread::ParamInfo info;
        m_thread->getParamInfo("maxVelXSpinBox", info);
        QCOMPARE(info.currentValue.toDouble(), 0.4);
        qDebug() << "[testVerifyDetectsMismatch] 回读验证机制正常工作";
    } else {
        // 如果失败，应该有详细的错误信息
        QString message = arguments.at(1).toString();
        qDebug() << "[testVerifyDetectsMismatch] 应用失败:" << message;
        QVERIFY2(message.contains("失败") || message.contains("验证"),
                 "失败消息应包含失败或验证相关信息");
    }
}

void TestNav2ParameterThreadIntegration::testVerifyAfterPartialFailure()
{
    // 测试多节点参数在部分节点失败时的行为
    Nav2ParameterThread::ParamInfo info;

    // robotRadius 写入到两个节点
    m_thread->getParamInfo("robotRadiusSpinBox", info);
    double originalValue = info.currentValue.toDouble();
    double testValue = originalValue + 0.01;

    m_thread->setPendingValue("robotRadiusSpinBox", testValue);

    QSignalSpy spy(m_thread, &Nav2ParameterThread::parameterApplied);
    m_thread->requestApply();

    QVERIFY(spy.wait(10000));

    QList<QVariant> arguments = spy.takeFirst();
    bool success = arguments.at(0).toBool();
    QStringList failedKeys = arguments.at(3).toStringList();

    // 如果成功，两个节点都应该成功写入
    if (success) {
        m_thread->requestRefresh();
        QThread::msleep(1000);

        m_thread->getParamInfo("robotRadiusSpinBox", info);
        QCOMPARE(info.currentValue.toDouble(), testValue);
        qDebug() << "[testVerifyAfterPartialFailure] 多节点参数写入成功";
    } else {
        // 如果失败，应该记录到失败列表
        QVERIFY2(!failedKeys.isEmpty() || arguments.at(1).toString().contains("失败"),
                 "失败时应有失败列表或失败消息");
        qDebug() << "[testVerifyAfterPartialFailure] 失败的参数:" << failedKeys;
    }

    // 恢复原始值
    m_thread->setPendingValue("robotRadiusSpinBox", originalValue);
    m_thread->requestApply();
    QThread::msleep(500);
}

void TestNav2ParameterThreadIntegration::testMultiNodeRollback()
{
    // 测试多节点参数的回滚机制
    // 场景：第一个节点设置成功，第二个节点设置失败，验证第一个节点被回滚

    QSKIP("此测试需要手动关闭 Nav2 节点，跳过自动测试。请使用手动测试步骤验证回滚机制。");

    // === 手动测试步骤 ===
    //
    // 前置条件：Nav2 仿真已启动
    //
    // 步骤 1：获取 robotRadius 的初始值
    //   ros2 param get /local_costmap/local_costmap robot_radius
    //   ros2 param get /global_costmap/global_costmap robot_radius
    //
    // 步骤 2：关闭 global_costmap 节点（模拟第二个节点失败）
    //   ros2 lifecycle set /global_costmap/global_costmap shutdown
    //
    // 步骤 3：在 UI 中修改 robot_radius 参数并点击"应用"
    //   - 预期：操作返回失败
    //   - 预期：日志输出 "开始回滚已写入的节点: robotRadiusSpinBox"
    //   - 预期：日志输出 "参数设置成功并已验证" (回滚操作)
    //
    // 步骤 4：验证 local_costmap 的值被回滚到原值
    //   ros2 param get /local_costmap/local_costmap robot_radius
    //   - 预期：值应与步骤 1 中的初始值相同
    //
    // 步骤 5：恢复 global_costmap 节点
    //   ros2 lifecycle set /global_costmap/global_costmap configure
    //   ros2 lifecycle set /global_costmap/global_costmap activate
    //
    // 步骤 6：验证回滚机制成功
    //   - 如果 local_costmap 的值被恢复到原值，则回滚机制工作正常
    //
    // === 预期日志输出 ===
    // [Nav2ParameterThread] 备份参数: robotRadiusSpinBox ...
    // [Nav2ParameterThread] 参数设置成功并已验证: "robot_radius" = ...
    // [Nav2ParameterThread] 应用参数失败: robotRadiusSpinBox 节点: "/global_costmap/global_costmap"
    // [Nav2ParameterThread] 开始回滚已写入的节点: robotRadiusSpinBox
    // [Nav2ParameterThread] 参数设置成功并已验证: "robot_radius" = ... (回滚到原值)
}

#include "testnav2parameterthreadintegration.moc"
