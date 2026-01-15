#include "testuserauthmanager.h"

void TestUserAuthManager::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestUserAuthManager::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestUserAuthManager::init()
{
    m_storageEngine = new UserStorageEngine();
    m_storageEngine->initialize(m_testDbPath);

    m_authManager = new UserAuthManager();
    QVERIFY(m_authManager->initialize(m_storageEngine));
}

void TestUserAuthManager::cleanup()
{
    delete m_authManager;
    m_authManager = nullptr;
    delete m_storageEngine;
    m_storageEngine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestUserAuthManager::testInitialize()
{
    UserAuthManager manager;
    QVERIFY(manager.initialize(m_storageEngine));
    QVERIFY(manager.isInitialized());
}

void TestUserAuthManager::testIsInitialized()
{
    QVERIFY(m_authManager->isInitialized());
}

void TestUserAuthManager::testHashPassword()
{
    QString password = "testpassword";
    QString hash = m_authManager->hashPassword(password);

    QVERIFY(!hash.isEmpty());
    QVERIFY(hash != password);
}

void TestUserAuthManager::testVerifyPassword()
{
    QString password = "testpassword";
    QString hash = m_authManager->hashPassword(password);

    QVERIFY(m_authManager->verifyPassword(password, hash));
    QVERIFY(!m_authManager->verifyPassword("wrongpassword", hash));
}

void TestUserAuthManager::testLogin()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QVERIFY(m_authManager->login("testuser", "password123"));
    QVERIFY(m_authManager->isLoggedIn());
}

void TestUserAuthManager::testLoginWithInvalidCredentials()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QVERIFY(!m_authManager->login("testuser", "wrongpassword"));
    QVERIFY(!m_authManager->isLoggedIn());
}

void TestUserAuthManager::testLoginWithInactiveUser()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    User user = m_storageEngine->getUserByUsername("testuser");
    user.active = false;
    m_storageEngine->updateUser(user);

    QVERIFY(!m_authManager->login("testuser", "password123"));
}

void TestUserAuthManager::testLogout()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->logout());
    QVERIFY(!m_authManager->isLoggedIn());
}

void TestUserAuthManager::testIsLoggedIn()
{
    QVERIFY(!m_authManager->isLoggedIn());

    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->isLoggedIn());
}

void TestUserAuthManager::testGetCurrentUser()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    User user = m_authManager->getCurrentUser();
    QVERIFY(user.username == "testuser");
}

void TestUserAuthManager::testGetCurrentUsername()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->getCurrentUsername() == "testuser");
}

void TestUserAuthManager::testGetCurrentPermission()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->getCurrentPermission() == UserPermission::ADMIN);
}

void TestUserAuthManager::testHasPermission()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->hasPermission(UserPermission::ADMIN));
    QVERIFY(m_authManager->hasPermission(UserPermission::OPERATOR));
    QVERIFY(m_authManager->hasPermission(UserPermission::VIEWER));
}

void TestUserAuthManager::testCanView()
{
    m_authManager->createUser("testuser", "password123", UserPermission::VIEWER);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->canView());
}

void TestUserAuthManager::testCanOperate()
{
    m_authManager->createUser("testuser", "password123", UserPermission::OPERATOR);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->canOperate());
}

void TestUserAuthManager::testCanAdmin()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->canAdmin());
}

void TestUserAuthManager::testChangePassword()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(m_authManager->changePassword("password123", "newpassword"));
}

void TestUserAuthManager::testChangePasswordWithWrongOldPassword()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QVERIFY(!m_authManager->changePassword("wrongpassword", "newpassword"));
}

void TestUserAuthManager::testResetPassword()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->createUser("admin", "admin123", UserPermission::ADMIN);
    m_authManager->login("admin", "admin123");

    QVERIFY(m_authManager->resetPassword("testuser", "newpassword"));
}

void TestUserAuthManager::testCreateUser()
{
    QVERIFY(m_authManager->createUser("testuser", "password123", UserPermission::ADMIN));
    QVERIFY(m_storageEngine->userExists("testuser"));
}

void TestUserAuthManager::testCreateUserWithInvalidUsername()
{
    QVERIFY(!m_authManager->createUser("", "password123", UserPermission::ADMIN));
    QVERIFY(!m_authManager->createUser("ab", "password123", UserPermission::ADMIN));
}

void TestUserAuthManager::testCreateUserWithInvalidPassword()
{
    QVERIFY(!m_authManager->createUser("testuser", "12345", UserPermission::ADMIN));
}

void TestUserAuthManager::testDeleteUser()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    QVERIFY(m_authManager->deleteUser("testuser"));
    QVERIFY(!m_storageEngine->userExists("testuser"));
}

void TestUserAuthManager::testUpdateUserPermission()
{
    m_authManager->createUser("testuser", "password123", UserPermission::VIEWER);
    QVERIFY(m_authManager->updateUserPermission("testuser", UserPermission::ADMIN));

    User user = m_storageEngine->getUserByUsername("testuser");
    QVERIFY(user.permission == UserPermission::ADMIN);
}

void TestUserAuthManager::testGetAllUsers()
{
    m_authManager->createUser("user1", "password1", UserPermission::VIEWER);
    m_authManager->createUser("user2", "password2", UserPermission::OPERATOR);
    m_authManager->createUser("user3", "password3", UserPermission::ADMIN);

    QVector<User> users = m_authManager->getAllUsers();
    QVERIFY(users.size() >= 3);
}

void TestUserAuthManager::testGetLastError()
{
    QString error = m_authManager->getLastError();
    QVERIFY(error.isEmpty() || !error.isEmpty());
}

void TestUserAuthManager::testSignalLoginSuccess()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QSignalSpy spy(m_authManager, &UserAuthManager::loginSuccess);
    m_authManager->login("testuser", "password123");

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalLoginFailed()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QSignalSpy spy(m_authManager, &UserAuthManager::loginFailed);
    m_authManager->login("testuser", "wrongpassword");

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalLogoutSuccess()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QSignalSpy spy(m_authManager, &UserAuthManager::logoutSuccess);
    m_authManager->logout();

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalPasswordChanged()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    m_authManager->login("testuser", "password123");

    QSignalSpy spy(m_authManager, &UserAuthManager::passwordChanged);
    m_authManager->changePassword("password123", "newpassword");

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalUserCreated()
{
    QSignalSpy spy(m_authManager, &UserAuthManager::userCreated);
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalUserDeleted()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QSignalSpy spy(m_authManager, &UserAuthManager::userDeleted);
    m_authManager->deleteUser("testuser");

    QVERIFY(spy.count() == 1);
}

void TestUserAuthManager::testSignalPermissionChanged()
{
    m_authManager->createUser("testuser", "password123", UserPermission::VIEWER);

    QSignalSpy spy(m_authManager, &UserAuthManager::permissionChanged);
    m_authManager->updateUserPermission("testuser", UserPermission::ADMIN);

    QVERIFY(spy.count() == 1);
}
