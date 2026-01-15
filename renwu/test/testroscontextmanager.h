#ifndef TESTROSCONTEXTMANAGER_H
#define TESTROSCONTEXTMANAGER_H

#include <QtTest/QtTest>
#include <QThread>
#include "roscontextmanager.h"

class TestROSContextManager : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testGetInstance();
    void testInitialize();
    void testIsInitialized();
    void testGetContext();
    void testShutdown();
    void testMultipleInitialize();
    void testThreadSafety();

private:
    ROSContextManager* m_manager;
};

#endif
