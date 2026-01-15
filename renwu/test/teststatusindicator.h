#ifndef TESTSTATUSINDICATOR_H
#define TESTSTATUSINDICATOR_H

#include <QtTest/QtTest>
#include <QDateTime>
#include <QPointF>
#include "statusindicator.h"

class TestStatusIndicator : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testIndicatorConstructor();
    void testIndicatorManagerConstructor();
    void testAddIndicator();
    void testRemoveIndicator();
    void testGetIndicators();
    void testClear();
    void testGetIndicatorsByType();
    void testIndicatorInfoType();
    void testIndicatorWarningType();
    void testIndicatorErrorType();
    void testIndicatorPosition();
    void testIndicatorMessage();
    void testIndicatorTimestamp();

private:
    StatusIndicatorManager* m_manager;
};

#endif
