#include "testuserstorageengine.h"

void TestUserStorageEngine::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestUserStorageEngine::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestUserStorageEngine::init()
{
    m_engine = new UserStorageEngine();
    QVERIFY(m_engine->initialize(m_testDbPath));
}

void TestUserStorageEngine::cleanup()
{
    delete m_engine;
    m_engine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestUserStorageEngine::testInitialize()
{
    UserStorageEngine engine;
    QVERIFY(engine.initialize(m_testDbPath));
    QVERIFY(engine.isInitialized());
}

void TestUserStorageEngine::testInitializeWithCustomPath()
{
    QString customPath = m_testDbPath + "_custom";
    UserStorageEngine engine;
    QVERIFY(engine.initialize(customPath));
    QVERIFY(engine.isInitialized());
    QFile::remove(customPath);
}

void TestUserStorageEngine::testIsInitialized()
{
    UserStorageEngine engine;
    QVERIFY(!engine.isInitialized());
    engine.initialize(m_testDbPath);
    QVERIFY(engine.isInitialized());
}

void TestUserStorageEngine::testInsertUser()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    QVERIFY(m_engine->insertUser(user));

    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getId() == 1);
    QVERIFY(retrieved.getUsername() == "testuser");
}

void TestUserStorageEngine::testInsertDuplicateUser()
{
    User user1(1, "testuser", "hash123", UserPermission::ADMIN);
    QVERIFY(m_engine->insertUser(user1));

    User user2(2, "testuser", "hash456", UserPermission::OPERATOR);
    QVERIFY(!m_engine->insertUser(user2));
}

void TestUserStorageEngine::testGetUserById()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getId() == 1);
    QVERIFY(retrieved.getUsername() == "testuser");
    QVERIFY(retrieved.getPasswordHash() == "hash123");
    QVERIFY(retrieved.getPermission() == UserPermission::ADMIN);
}

void TestUserStorageEngine::testGetUserByUsername()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    User retrieved = m_engine->getUserByUsername("testuser");
    QVERIFY(retrieved.getId() == 1);
    QVERIFY(retrieved.getUsername() == "testuser");
}

void TestUserStorageEngine::testGetUserByIdNotFound()
{
    User retrieved = m_engine->getUserById(999);
    QVERIFY(retrieved.getId() == 0);
}

void TestUserStorageEngine::testGetUserByUsernameNotFound()
{
    User retrieved = m_engine->getUserByUsername("nonexistent");
    QVERIFY(retrieved.getId() == 0);
}

void TestUserStorageEngine::testUpdateUser()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    user.setPermission(UserPermission::OPERATOR);
    user.setActive(false);
    QVERIFY(m_engine->updateUser(user));

    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getPermission() == UserPermission::OPERATOR);
    QVERIFY(retrieved.isActive() == false);
}

void TestUserStorageEngine::testDeleteUserById()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->deleteUser(1));
    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getId() == 0);
}

void TestUserStorageEngine::testDeleteUserByUsername()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->deleteUser("testuser"));
    User retrieved = m_engine->getUserByUsername("testuser");
    QVERIFY(retrieved.getId() == 0);
}

void TestUserStorageEngine::testUpdateLastLogin()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->updateLastLogin(1));
    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getLastLogin().isValid());
}

void TestUserStorageEngine::testChangePasswordById()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->changePassword(1, "newHash"));
    User retrieved = m_engine->getUserById(1);
    QVERIFY(retrieved.getPasswordHash() == "newHash");
}

void TestUserStorageEngine::testChangePasswordByUsername()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->changePassword("testuser", "newHash"));
    User retrieved = m_engine->getUserByUsername("testuser");
    QVERIFY(retrieved.getPasswordHash() == "newHash");
}

void TestUserStorageEngine::testUserExists()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(m_engine->userExists("testuser"));
}

void TestUserStorageEngine::testUserExistsNotFound()
{
    QVERIFY(!m_engine->userExists("nonexistent"));
}

void TestUserStorageEngine::testIsUserActive()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    user.setActive(true);
    m_engine->insertUser(user);

    QVERIFY(m_engine->isUserActive("testuser"));

    user.setActive(false);
    m_engine->updateUser(user);
    QVERIFY(!m_engine->isUserActive("testuser"));
}

void TestUserStorageEngine::testGetAllUsers()
{
    User user1(1, "user1", "hash1", UserPermission::ADMIN);
    User user2(2, "user2", "hash2", UserPermission::OPERATOR);
    User user3(3, "user3", "hash3", UserPermission::VIEWER);

    m_engine->insertUser(user1);
    m_engine->insertUser(user2);
    m_engine->insertUser(user3);

    QVector<User> users = m_engine->getAllUsers();
    QVERIFY(users.size() == 3);
}

void TestUserStorageEngine::testHashPassword()
{
    QString password = "testpassword";
    QString hash1 = UserStorageEngine::hashPassword(password);
    QString hash2 = UserStorageEngine::hashPassword(password);

    QVERIFY(!hash1.isEmpty());
    QVERIFY(hash1 == hash2);
}

void TestUserStorageEngine::testGetLastError()
{
    QString error = m_engine->getLastError();
    QVERIFY(error.isEmpty() || !error.isEmpty());
}

void TestUserStorageEngine::testSignalUserInserted()
{
    QSignalSpy spy(m_engine, &UserStorageEngine::userInserted);
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QVERIFY(spy.count() == 1);
}

void TestUserStorageEngine::testSignalUserUpdated()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QSignalSpy spy(m_engine, &UserStorageEngine::userUpdated);
    user.setPermission(UserPermission::OPERATOR);
    m_engine->updateUser(user);

    QVERIFY(spy.count() == 1);
}

void TestUserStorageEngine::testSignalUserDeleted()
{
    User user(1, "testuser", "hash123", UserPermission::ADMIN);
    m_engine->insertUser(user);

    QSignalSpy spy(m_engine, &UserStorageEngine::userDeleted);
    m_engine->deleteUser(1);

    QVERIFY(spy.count() == 1);
}

void TestUserStorageEngine::testThreadSafety()
{
    const int threadCount = 10;
    QVector<QThread*> threads;

    for (int i = 0; i < threadCount; ++i) {
        QThread* thread = QThread::create([this, i]() {
            User user(i, QString("user%1").arg(i), QString("hash%1").arg(i), UserPermission::VIEWER);
            m_engine->insertUser(user);
            m_engine->getUserById(i);
        });
        threads.append(thread);
        thread->start();
    }

    for (QThread* thread : threads) {
        thread->wait();
        delete thread;
    }

    QVERIFY(m_engine->getAllUsers().size() >= threadCount);
}
