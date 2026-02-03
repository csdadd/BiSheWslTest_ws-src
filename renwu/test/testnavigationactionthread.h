#ifndef TESTNAVIGATIONACTIONTHREAD_H
#define TESTNAVIGATIONACTIONTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "navigationactionthread.h"

class TestNavigationActionThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    // 基础功能测试
    void testConstructor();
    void testDestructor();

    // 线程生命周期测试
    void testThreadStart();
    void testThreadStop();
    void testIsRunning();
    void testThreadStartedSignal();
    void testThreadStoppedSignal();

    // 导航方法测试
    void testSendGoalToPose();
    void testSendGoalToPoseWithDifferentCoordinates();
    void testSendGoalToPoseWithYaw();
    void testCancelCurrentGoal();
    void testCancelGoalWithoutNavigation();
    void testIsNavigating();
    void testIsNavigatingWithoutGoal();
    void testGetCurrentGoal();
    void testGetCurrentGoalWithoutGoal();

    // 信号测试
    void testGoalAcceptedSignal();
    void testGoalRejectedSignal();
    void testFeedbackReceivedSignal();
    void testResultReceivedSignal();
    void testGoalCanceledSignal();

private:
    NavigationActionThread* m_thread;
};

#endif
