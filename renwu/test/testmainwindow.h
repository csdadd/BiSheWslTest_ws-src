#ifndef TESTMAINWINDOW_H
#define TESTMAINWINDOW_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QDateTimeEdit>
#include <QTableView>
#include <QAction>
#include "mainwindow.h"

class TestMainWindow : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testUIComponents();
    void testThreadInitialization();
    void testSignalConnections();
    void testBatteryStatusSlot();
    void testPositionSlot();
    void testOdometrySlot();
    void testSystemTimeSlot();
    void testDiagnosticsSlot();
    void testNavigationStatusSlot();
    void testNavigationFeedbackSlot();
    void testNavigationPathSlot();
    void testLogMessageSlot();
    void testCollisionDetectedSlot();
    void testAnomalyDetectedSlot();
    void testBehaviorTreeLogSlot();
    void testMapReceivedSlot();
    void testMapClickedSlot();
    void testNavigationControlSlots();
    void testUserPermissionSlots();
    void testUIPermissionControl();
    void testThreadLifecycle();

    void testLogFilterCheckBoxesUI();
    void testLogFiltering();
    void testRefreshLogDisplay();
    void testOnFilterChanged();
    void testLogRealTimeFiltering();

private:
    MainWindow* m_mainWindow;
};

#endif
