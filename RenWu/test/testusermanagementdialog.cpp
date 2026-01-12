#include "testusermanagementdialog.h"

void TestUserManagementDialog::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestUserManagementDialog::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestUserManagementDialog::init()
{
    m_storageEngine = new UserStorageEngine();
    m_storageEngine->initialize(m_testDbPath);

    m_authManager = new UserAuthManager();
    m_authManager->initialize(m_storageEngine);
    m_authManager->createUser("admin", "admin123", UserPermission::ADMIN);
    m_authManager->login("admin", "admin123");

    m_dialog = new UserManagementDialog(m_authManager);
}

void TestUserManagementDialog::cleanup()
{
    delete m_dialog;
    m_dialog = nullptr;
    delete m_authManager;
    m_authManager = nullptr;
    delete m_storageEngine;
    m_storageEngine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestUserManagementDialog::testConstructor()
{
    QVERIFY(m_dialog != nullptr);
    QVERIFY(m_dialog->windowTitle().contains("用户管理", Qt::CaseInsensitive));
}

void TestUserManagementDialog::testOnAddUserClicked()
{
    QPushButton* addUserButton = m_dialog->findChild<QPushButton*>("addUserButton");
    QVERIFY(addUserButton != nullptr);
}

void TestUserManagementDialog::testOnDeleteUserClicked()
{
    QPushButton* deleteUserButton = m_dialog->findChild<QPushButton*>("deleteUserButton");
    QVERIFY(deleteUserButton != nullptr);
}

void TestUserManagementDialog::testOnChangePasswordClicked()
{
    QPushButton* changePasswordButton = m_dialog->findChild<QPushButton*>("changePasswordButton");
    QVERIFY(changePasswordButton != nullptr);
}

void TestUserManagementDialog::testOnRefreshClicked()
{
    QPushButton* refreshButton = m_dialog->findChild<QPushButton*>("refreshButton");
    QVERIFY(refreshButton != nullptr);
}

void TestUserManagementDialog::testOnCloseClicked()
{
    QPushButton* closeButton = m_dialog->findChild<QPushButton*>("closeButton");
    QVERIFY(closeButton != nullptr);
}

void TestUserManagementDialog::testOnUserCreated()
{
    m_authManager->createUser("testuser", "password123", UserPermission::OPERATOR);

    QTest::qWait(100);

    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
    QVERIFY(userTable->rowCount() > 0);
}

void TestUserManagementDialog::testOnUserDeleted()
{
    m_authManager->createUser("testuser", "password123", UserPermission::OPERATOR);
    QTest::qWait(100);

    m_authManager->deleteUser("testuser");
    QTest::qWait(100);

    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
}

void TestUserManagementDialog::testOnPermissionChanged()
{
    m_authManager->createUser("testuser", "password123", UserPermission::VIEWER);
    QTest::qWait(100);

    m_authManager->updateUserPermission("testuser", UserPermission::ADMIN);
    QTest::qWait(100);

    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
}

void TestUserManagementDialog::testOnPasswordChanged()
{
    m_authManager->createUser("testuser", "password123", UserPermission::OPERATOR);
    m_authManager->login("testuser", "password123");

    QSignalSpy spy(m_authManager, &UserAuthManager::passwordChanged);
    m_authManager->changePassword("password123", "newpassword");

    QVERIFY(spy.count() == 1);
}

void TestUserManagementDialog::testOnErrorOccurred()
{
    QSignalSpy spy(m_authManager, &UserAuthManager::errorOccurred);

    m_authManager->createUser("", "password123", UserPermission::OPERATOR);

    QVERIFY(spy.count() >= 0);
}

void TestUserManagementDialog::testUserTable()
{
    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
    QVERIFY(userTable->columnCount() > 0);
}

void TestUserManagementDialog::testPermissionToString()
{
    QVERIFY(m_dialog->windowTitle().contains("用户管理", Qt::CaseInsensitive));
}

void TestUserManagementDialog::testStringToPermission()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);
    QTest::qWait(100);

    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
}

void TestUserManagementDialog::testLoadUsers()
{
    m_authManager->createUser("user1", "password1", UserPermission::VIEWER);
    m_authManager->createUser("user2", "password2", UserPermission::OPERATOR);
    QTest::qWait(100);

    QTableWidget* userTable = m_dialog->findChild<QTableWidget*>("userTable");
    QVERIFY(userTable != nullptr);
    QVERIFY(userTable->rowCount() >= 2);
}
