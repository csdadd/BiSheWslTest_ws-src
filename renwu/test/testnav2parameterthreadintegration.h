#ifndef TESTNAV2PARAMETERTHREADINTEGRATION_H
#define TESTNAV2PARAMETERTHREADINTEGRATION_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QThread>
#include "nav2parameterthread.h"
#include "roscontextmanager.h"

class TestNav2ParameterThreadIntegration : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    // 参数读取测试
    void testReadNormalParameters();
    void testReadArrayParameters();
    void testReadCostmapParameters();
    void testReadRobotRadius();
    void testAllParametersRead();

    // 参数写入测试
    void testWriteNormalParameter();
    void testWriteArrayParameter();
    void testWriteRobotRadius();
    void testWriteBatchParameters();
    void testWriteWithNoModifications();

    // 刷新功能测试
    void testRefreshAllParameters();
    void testRefreshPreservesDefault();

    // 重置功能测试
    void testResetAllParameters();
    void testResetThenApply();

    // 放弃功能测试
    void testDiscardSingleParameter();
    void testDiscardMultipleParameters();
    void testDiscardNoModifications();

    // 线程安全测试
    void testConcurrentRefreshAndModify();
    void testRapidSequentialOperations();

    // 异常处理测试
    void testNodeUnavailable();

    // 信号测试
    void testOperationFinishedSignal();
    void testParameterRefreshedSignalSuccess();
    void testParameterRefreshedSignalFailure();
    void testParameterAppliedSignalWithMultipleParams();
    void testOperationProgressSignalDuringRefresh();

    // velocity_smoother 数组参数测试
    void testVelocitySmootherArrayParameterWrite();

    // 多节点参数测试
    void testMultiNodeParameterConsistency();

    // 默认值保留测试
    void testDefaultValuePreservedAfterModify();

    // 完整工作流测试
    void testFullWorkflowModifyApplyRefresh();
    void testFullWorkflowModifyResetApply();
    void testFullWorkflowModifyDiscard();

    // 回读验证测试 (新增)
    void testVerifyParameterValueAfterWrite();
    void testVerifyArrayParameterAfterWrite();
    void testVerifyDetectsMismatch();
    void testVerifyAfterPartialFailure();

    // 回滚机制测试 (新增)
    void testMultiNodeRollback();

private:
    Nav2ParameterThread* m_thread;
    QMap<QString, QVariant> m_initialValues;  // 保存初始值用于恢复
};

#endif
