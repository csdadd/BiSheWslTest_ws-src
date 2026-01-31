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

    QVERIFY(user.getId() == 0);
    QVERIFY(user.getUsername().isEmpty());
    QVERIFY(user.getPasswordHash().isEmpty());
    QVERIFY(user.getPermission() == UserPermission::VIEWER);
    QVERIFY(user.isActive() == true);
}

void TestUser::testParameterizedConstructor()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);

    QVERIFY(user.getId() == 1);
    QVERIFY(user.getUsername() == "testuser");
    QVERIFY(user.getPasswordHash() == "hash123");
    QVERIFY(user.getPermission() == UserPermission::ADMIN);
    QVERIFY(user.isActive() == true);
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
    user.setId(5);
    user.setUsername("admin");
    user.setPasswordHash("passwordHash");
    user.setPermission(UserPermission::OPERATOR);
    user.setCreatedAt(QDateTime::currentDateTime());
    user.setLastLogin(QDateTime::currentDateTime());
    user.setActive(false);

    QVERIFY(user.getId() == 5);
    QVERIFY(user.getUsername() == "admin");
    QVERIFY(user.getPasswordHash() == "passwordHash");
    QVERIFY(user.getPermission() == UserPermission::OPERATOR);
    QVERIFY(user.getCreatedAt().isValid());
    QVERIFY(user.getLastLogin().isValid());
    QVERIFY(user.isActive() == false);
}

void TestUser::testUserCopy()
{
    User user1(1, "testuser", "hash123", UserPermission::ADMIN);
    user1.setCreatedAt(QDateTime::currentDateTime());
    user1.setLastLogin(QDateTime::currentDateTime());

    User user2 = user1;

    QVERIFY(user2.getId() == user1.getId());
    QVERIFY(user2.getUsername() == user1.getUsername());
    QVERIFY(user2.getPasswordHash() == user1.getPasswordHash());
    QVERIFY(user2.getPermission() == user1.getPermission());
    QVERIFY(user2.isActive() == user1.isActive());
}
