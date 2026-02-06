// ==================== DEPRECATED ====================
// 此测试文件已废弃，对应的源文件 navigationactionclient.h/cpp 已废弃
// 废弃原因：已被 NavigationActionThread（独立线程模式）替代
// 保留原因：作为工作量证明保留
// ================================================

#ifndef TESTNAVIGATIONACTIONCLIENT_H
#define TESTNAVIGATIONACTIONCLIENT_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include <QTimer>
#include "navigationactionclient.h"

class TestNavigationActionClient : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testSendGoal();
    void testSendGoalWithDifferentCoordinates();
    void testSendGoalWithYaw();
    void testCancelGoal();
    void testCancelGoalWithoutNavigation();
    void testIsNavigating();
    void testIsNavigatingWithoutGoal();
    void testGetCurrentGoal();
    void testGetCurrentGoalWithoutGoal();
    void testGoalAcceptedSignal();
    void testGoalRejectedSignal();
    void testFeedbackReceivedSignal();
    void testResultReceivedSignal();
    void testGoalCanceledSignal();
};

#endif
