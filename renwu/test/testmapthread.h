// ==================== DEPRECATED ====================
// 此测试文件已废弃，对应的源文件 mapthread.h/cpp 已废弃
// 废弃原因：项目改用固定地图图片，不再订阅实时地图
// 保留原因：作为工作量证明保留
// ================================================

#ifndef TESTMAPTHREAD_H
#define TESTMAPTHREAD_H

#include <QtTest/QtTest>
#include <QSignalSpy>
#include "mapthread.h"

class TestMapThread : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testInitialize();
    void testStart();
    void testStop();
    void testMapReceivedSignal();
    void testConnectionStateSignal();
    void testThreadLifecycle();

private:
    MapThread* m_thread;
};

#endif
