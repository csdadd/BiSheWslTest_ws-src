#ifndef TESTUSERMANAGEMENTDIALOG_H
#define TESTUSERMANAGEMENTDIALOG_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include "usermanagementdialog.h"
#include "userauthmanager.h"
#include "userstorageengine.h"

class TestUserManagementDialog : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testOnAddUserClicked();
    void testOnDeleteUserClicked();
    void testOnChangePasswordClicked();
    void testOnRefreshClicked();
    void testOnCloseClicked();
    void testOnUserCreated();
    void testOnUserDeleted();
    void testOnPermissionChanged();
    void testOnPasswordChanged();
    void testOnErrorOccurred();
    void testUserTable();
    void testPermissionToString();
    void testStringToPermission();
    void testLoadUsers();

private:
    QString m_testDbPath;
    UserStorageEngine* m_storageEngine;
    UserAuthManager* m_authManager;
    UserManagementDialog* m_dialog;
};

#endif
