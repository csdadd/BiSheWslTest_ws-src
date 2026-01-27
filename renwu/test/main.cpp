#include <QtTest/QtTest>
#include "testmapconverter.h"
#include "testmapwidget.h"
#include "testintegration.h"
#include "testsystem.h"
#include "testmaploader.h"
#include "testnavigationactionclient.h"
#include "testpathvisualizer.h"
#include "testnavigationintegration.h"
#include "testuser.h"
#include "testuserstorageengine.h"
#include "testuserauthmanager.h"
#include "testlogindialog.h"
#include "testusermanagementdialog.h"
#include "testlogstorageengine.h"
#include "testlogtablemodel.h"
#include "testlogfilterproxymodel.h"
#include "testlogquerytask.h"
#include "testlogthread.h"
#include "testbasethread.h"
#include "testroscontextmanager.h"
#include "testthreadsafequeue.h"
#include "testrobotstatusthread.h"
#include "testnavstatusthread.h"
#include "testsystemmonitorthread.h"
#include "testinfothread.h"
#include "testmapmarker.h"
#include "teststatusindicator.h"
#include "testmapcache.h"
#include "testmapthread.h"
#include "testmainwindow.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    int result = 0;

    {
        TestMapConverter testMapConverter;
        result |= QTest::qExec(&testMapConverter, argc, argv);
    }

    {
        TestMapWidget testMapWidget;
        result |= QTest::qExec(&testMapWidget, argc, argv);
    }

    {
        TestIntegration testIntegration;
        result |= QTest::qExec(&testIntegration, argc, argv);
    }

    {
        TestSystem testSystem;
        result |= QTest::qExec(&testSystem, argc, argv);
    }

    {
        TestMapLoader testMapLoader;
        result |= QTest::qExec(&testMapLoader, argc, argv);
    }

    {
        TestNavigationActionClient testNavigationActionClient;
        result |= QTest::qExec(&testNavigationActionClient, argc, argv);
    }

    {
        TestPathVisualizer testPathVisualizer;
        result |= QTest::qExec(&testPathVisualizer, argc, argv);
    }

    {
        TestNavigationIntegration testNavigationIntegration;
        result |= QTest::qExec(&testNavigationIntegration, argc, argv);
    }

    {
        TestUser testUser;
        result |= QTest::qExec(&testUser, argc, argv);
    }

    {
        TestUserStorageEngine testUserStorageEngine;
        result |= QTest::qExec(&testUserStorageEngine, argc, argv);
    }

    {
        TestUserAuthManager testUserAuthManager;
        result |= QTest::qExec(&testUserAuthManager, argc, argv);
    }

    {
        TestLoginDialog testLoginDialog;
        result |= QTest::qExec(&testLoginDialog, argc, argv);
    }

    {
        TestUserManagementDialog testUserManagementDialog;
        result |= QTest::qExec(&testUserManagementDialog, argc, argv);
    }

    {
        TestLogStorageEngine testLogStorageEngine;
        result |= QTest::qExec(&testLogStorageEngine, argc, argv);
    }

    {
        TestLogTableModel testLogTableModel;
        result |= QTest::qExec(&testLogTableModel, argc, argv);
    }

    {
        TestLogFilterProxyModel testLogFilterProxyModel;
        result |= QTest::qExec(&testLogFilterProxyModel, argc, argv);
    }

    {
        TestLogQueryTask testLogQueryTask;
        result |= QTest::qExec(&testLogQueryTask, argc, argv);
    }

    {
        TestLogThread testLogThread;
        result |= QTest::qExec(&testLogThread, argc, argv);
    }

    {
        TestBaseThread testBaseThread;
        result |= QTest::qExec(&testBaseThread, argc, argv);
    }

    {
        TestROSContextManager testROSContextManager;
        result |= QTest::qExec(&testROSContextManager, argc, argv);
    }

    {
        TestThreadSafeQueue testThreadSafeQueue;
        result |= QTest::qExec(&testThreadSafeQueue, argc, argv);
    }

    {
        TestRobotStatusThread testRobotStatusThread;
        result |= QTest::qExec(&testRobotStatusThread, argc, argv);
    }

    {
        TestNavStatusThread testNavStatusThread;
        result |= QTest::qExec(&testNavStatusThread, argc, argv);
    }

    {
        TestSystemMonitorThread testSystemMonitorThread;
        result |= QTest::qExec(&testSystemMonitorThread, argc, argv);
    }

    {
        TestInfoThread testInfoThread;
        result |= QTest::qExec(&testInfoThread, argc, argv);
    }

    {
        TestMapMarker testMapMarker;
        result |= QTest::qExec(&testMapMarker, argc, argv);
    }

    {
        TestStatusIndicator testStatusIndicator;
        result |= QTest::qExec(&testStatusIndicator, argc, argv);
    }

    {
        TestMapCache testMapCache;
        result |= QTest::qExec(&testMapCache, argc, argv);
    }

    {
        TestMapThread testMapThread;
        result |= QTest::qExec(&testMapThread, argc, argv);
    }

    {
        TestMainWindow testMainWindow;
        result |= QTest::qExec(&testMainWindow, argc, argv);
    }

    {
        TestNavigationIntegration testNavigationIntegration;
        result |= QTest::qExec(&testNavigationIntegration, argc, argv);
    }

    return result;
}
