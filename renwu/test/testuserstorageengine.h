#ifndef TESTUSERSTORAGEENGINE_H
#define TESTUSERSTORAGEENGINE_H

#include <QtTest/QtTest>
#include <QTemporaryFile>
#include <QThread>
#include "userstorageengine.h"

class TestUserStorageEngine : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testInitialize();
    void testInitializeWithCustomPath();
    void testIsInitialized();
    void testInsertUser();
    void testInsertDuplicateUser();
    void testGetUserById();
    void testGetUserByUsername();
    void testGetUserByIdNotFound();
    void testGetUserByUsernameNotFound();
    void testUpdateUser();
    void testDeleteUserById();
    void testDeleteUserByUsername();
    void testUpdateLastLogin();
    void testChangePasswordById();
    void testChangePasswordByUsername();
    void testUserExists();
    void testUserExistsNotFound();
    void testIsUserActive();
    void testGetAllUsers();
    void testHashPassword();
    void testGetLastError();
    void testSignalUserInserted();
    void testSignalUserUpdated();
    void testSignalUserDeleted();
    void testThreadSafety();

private:
    QString m_testDbPath;
    UserStorageEngine* m_engine;
};

#endif
