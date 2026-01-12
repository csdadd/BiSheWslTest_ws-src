#ifndef TESTUSERAUTHMANAGER_H
#define TESTUSERAUTHMANAGER_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include "userauthmanager.h"
#include "userstorageengine.h"

class TestUserAuthManager : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testInitialize();
    void testIsInitialized();
    void testHashPassword();
    void testVerifyPassword();
    void testLogin();
    void testLoginWithInvalidCredentials();
    void testLoginWithInactiveUser();
    void testLogout();
    void testIsLoggedIn();
    void testGetCurrentUser();
    void testGetCurrentUsername();
    void testGetCurrentPermission();
    void testHasPermission();
    void testCanView();
    void testCanOperate();
    void testCanAdmin();
    void testChangePassword();
    void testChangePasswordWithWrongOldPassword();
    void testResetPassword();
    void testCreateUser();
    void testCreateUserWithInvalidUsername();
    void testCreateUserWithInvalidPassword();
    void testDeleteUser();
    void testUpdateUserPermission();
    void testGetAllUsers();
    void testGetLastError();
    void testSignalLoginSuccess();
    void testSignalLoginFailed();
    void testSignalLogoutSuccess();
    void testSignalPasswordChanged();
    void testSignalUserCreated();
    void testSignalUserDeleted();
    void testSignalPermissionChanged();

private:
    QString m_testDbPath;
    UserStorageEngine* m_storageEngine;
    UserAuthManager* m_authManager;
};

#endif
