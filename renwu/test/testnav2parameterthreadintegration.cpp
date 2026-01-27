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

#include "testnav2parameterthreadintegration.moc"
