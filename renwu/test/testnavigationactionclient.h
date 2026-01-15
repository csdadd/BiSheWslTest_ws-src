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
