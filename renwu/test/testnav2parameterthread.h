#ifndef TESTNAV2PARAMETERTHREAD_H
#define TESTNAV2PARAMETERTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QThread>
#include "nav2parameterthread.h"

class TestNav2ParameterThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    // 参数注册测试
    void testParameterCount();
    void testParameterMapping();
    void testNodeNames();
    void testRobotRadiusMultiNode();
    void testVelocitySmootherArrayParams();

    // 缓存机制测试
    void testGetParamInfo();
    void testGetParamInfoInvalidKey();
    void testGetAllParams();
    void testSetPendingValue();
    void testSetPendingValueInvalidKey();
    void testHasPendingChanges();
    void testHasPendingChangesNoModifications();
    void testMultipleModifiedParams();

    // 任务队列测试
    void testRequestRefresh();
    void testRequestApply();
    void testRequestReset();
    void testRequestDiscard();
    void testTaskQueueOrder();

    // 数据结构测试
    void testParamInfoPrimaryNode();
    void testParamInfoDefaultValues();
    void testParamTaskConstructor();

    // 参数转换测试 (P0)
    void testParameterToVariantDouble();
    void testParameterToVariantInt();
    void testParameterToVariantBool();
    void testParameterToVariantString();
    void testParameterToVariantDoubleArray();
    void testParameterToVariantDoubleArrayEmpty();
    void testParameterToVariantUnsupportedType();
    void testVariantToParameterDouble();
    void testVariantToParameterInt();
    void testVariantToParameterBool();
    void testVariantToString();
    void testVariantToParameterUnsupportedType();

    // 信号测试 (P0)
    void testParameterRefreshedSignal();
    void testParameterAppliedSignal();
    void testParameterAppliedSignalPartialFailure();
    void testOperationProgressSignal();

    // 边界值测试
    void testSetPendingValueZero();
    void testSetPendingValueNegative();
    void testSetPendingVeryLargeValue();
    void testParameterToVariantZero();

    // 状态一致性测试
    void testModifiedFlagAfterSetPending();
    void testModifiedFlagAfterDiscard();
    void testPendingValueAfterDiscard();

    // 数组参数测试
    void testArrayParameterYDirection();

private:
    Nav2ParameterThread* m_thread;
};

#endif
