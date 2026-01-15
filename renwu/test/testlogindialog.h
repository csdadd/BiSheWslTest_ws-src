#ifndef TESTLOGINDIALOG_H
#define TESTLOGINDIALOG_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QSignalSpy>
#include "logindialog.h"
#include "userauthmanager.h"
#include "userstorageengine.h"

class TestLoginDialog : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testConstructor();
    void testGetCurrentUser();
    void testOnLoginClicked();
    void testOnLoginClickedWithValidCredentials();
    void testOnLoginClickedWithInvalidCredentials();
    void testOnCancelClicked();
    void testOnLoginSuccess();
    void testOnLoginFailed();
    void testUsernameInput();
    void testPasswordInput();
    void testLoginButton();
    void testCancelButton();

private:
    QString m_testDbPath;
    UserStorageEngine* m_storageEngine;
    UserAuthManager* m_authManager;
    LoginDialog* m_dialog;
};

#endif
