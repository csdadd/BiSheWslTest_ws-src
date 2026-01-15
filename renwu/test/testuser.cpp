#include "testuser.h"

void TestUser::initTestCase()
{
}

void TestUser::cleanupTestCase()
{
}

void TestUser::init()
{
}

void TestUser::cleanup()
{
}

void TestUser::testDefaultConstructor()
{
    User user;

    QVERIFY(user.id == 0);
    QVERIFY(user.username.isEmpty());
    QVERIFY(user.passwordHash.isEmpty());
    QVERIFY(user.permission == UserPermission::VIEWER);
    QVERIFY(user.active == true);
}

void TestUser::testParameterizedConstructor()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);

    QVERIFY(user.id == 1);
    QVERIFY(user.username == "testuser");
    QVERIFY(user.passwordHash == "hash123");
    QVERIFY(user.permission == UserPermission::ADMIN);
    QVERIFY(user.active == true);
}

void TestUser::testPermissionEnumValues()
{
    QVERIFY(static_cast<int>(UserPermission::VIEWER) == 0);
    QVERIFY(static_cast<int>(UserPermission::OPERATOR) == 1);
    QVERIFY(static_cast<int>(UserPermission::ADMIN) == 2);
}

void TestUser::testUserFields()
{
    User user;
    user.id = 5;
    user.username = "admin";
    user.passwordHash = "passwordHash";
    user.permission = UserPermission::OPERATOR;
    user.createdAt = QDateTime::currentDateTime();
    user.lastLogin = QDateTime::currentDateTime();
    user.active = false;

    QVERIFY(user.id == 5);
    QVERIFY(user.username == "admin");
    QVERIFY(user.passwordHash == "passwordHash");
    QVERIFY(user.permission == UserPermission::OPERATOR);
    QVERIFY(user.createdAt.isValid());
    QVERIFY(user.lastLogin.isValid());
    QVERIFY(user.active == false);
}

void TestUser::testUserCopy()
{
    User user1(1, "testuser", "hash123", UserPermission::ADMIN);
    user1.createdAt = QDateTime::currentDateTime();
    user1.lastLogin = QDateTime::currentDateTime();

    User user2 = user1;

    QVERIFY(user2.id == user1.id);
    QVERIFY(user2.username == user1.username);
    QVERIFY(user2.passwordHash == user1.passwordHash);
    QVERIFY(user2.permission == user1.permission);
    QVERIFY(user2.active == user1.active);
}
