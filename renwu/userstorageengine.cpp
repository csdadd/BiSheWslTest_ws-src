#include "userstorageengine.h"
#include <QSqlQuery>
#include <QSqlError>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include <QCryptographicHash>
#include <QDateTime>

UserStorageEngine::UserStorageEngine(QObject* parent)
    : QObject(parent)
    , m_initialized(false)
{
}

UserStorageEngine::~UserStorageEngine()
{
    if (m_database.isOpen()) {
        m_database.close();
    }
}

bool UserStorageEngine::initialize(const QString& dbPath)
{
    QWriteLocker locker(&m_lock);

    if (m_initialized) {
        m_lastError = "Database already initialized";
        qDebug() << "[UserStorageEngine] Already initialized";
        return true;
    }

    qDebug() << "[UserStorageEngine] 正在初始化数据库...";

    if (dbPath.isEmpty()) {
        qDebug() << "[UserStorageEngine] 获取应用数据目录...";
        QString dataDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        qDebug() << "[UserStorageEngine] 数据目录:" << dataDir;

        QDir dir(dataDir);
        if (!dir.exists()) {
            qDebug() << "[UserStorageEngine] 创建数据目录...";
            if (!dir.mkpath(dataDir)) {
                qWarning() << "[UserStorageEngine] 警告：无法创建目录" << dataDir;
            }
        }
        m_dbPath = dataDir + "/users.db";
    } else {
        m_dbPath = dbPath;
    }

    qDebug() << "[UserStorageEngine] 数据库路径:" << m_dbPath;

    if (QSqlDatabase::contains("user_connection")) {
        qDebug() << "[UserStorageEngine] 使用现有数据库连接...";
        m_database = QSqlDatabase::database("user_connection");
    } else {
        qDebug() << "[UserStorageEngine] 创建新的数据库连接...";
        m_database = QSqlDatabase::addDatabase("QSQLITE", "user_connection");
    }

    m_database.setDatabaseName(m_dbPath);

    qDebug() << "[UserStorageEngine] 正在打开数据库...";
    if (!m_database.open()) {
        m_lastError = QString("Failed to open database: %1").arg(m_database.lastError().text());
        qCritical() << "[UserStorageEngine]" << m_lastError;
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[UserStorageEngine] 数据库打开成功，正在创建表...";
    if (!createTables()) {
        qCritical() << "[UserStorageEngine] 创建表失败";
        m_database.close();
        return false;
    }

    qDebug() << "[UserStorageEngine] 正在创建索引...";
    if (!createIndexes()) {
        qCritical() << "[UserStorageEngine] 创建索引失败";
        m_database.close();
        return false;
    }

    qDebug() << "[UserStorageEngine] 检查默认管理员账户...";

    // 直接检查而不使用带锁的 userExists()
    QSqlQuery checkQuery(m_database);
    checkQuery.prepare("SELECT COUNT(*) FROM users WHERE username = ?");
    checkQuery.addBindValue("admin");
    bool adminExists = false;
    if (checkQuery.exec() && checkQuery.next()) {
        adminExists = checkQuery.value(0).toInt() > 0;
    }

    if (!adminExists) {
        qDebug() << "[UserStorageEngine] 创建默认管理员账户...";

        // 直接插入数据，避免死锁
        QSqlQuery insertQuery(m_database);
        insertQuery.prepare(R"(
            INSERT INTO users (username, password_hash, permission, created_at, last_login, active)
            VALUES (?, ?, ?, ?, ?, ?)
        )");

        User defaultAdmin;
        defaultAdmin.username = "0";
        defaultAdmin.passwordHash = hashPassword("0");
        defaultAdmin.permission = UserPermission::ADMIN;
        defaultAdmin.createdAt = QDateTime::currentDateTime();
        defaultAdmin.active = true;

        insertQuery.addBindValue(defaultAdmin.username);
        insertQuery.addBindValue(defaultAdmin.passwordHash);
        insertQuery.addBindValue(static_cast<int>(defaultAdmin.permission));
        insertQuery.addBindValue(defaultAdmin.createdAt.toMSecsSinceEpoch());
        insertQuery.addBindValue(QVariant());
        insertQuery.addBindValue(defaultAdmin.active ? 1 : 0);

        if (!insertQuery.exec()) {
            qWarning() << "[UserStorageEngine] 创建默认管理员账户失败:" << insertQuery.lastError().text();
        } else {
            qDebug() << "[UserStorageEngine] 默认管理员账户已创建 (用户名: 0, 密码: 0)";
        }
    } else {
        qDebug() << "[UserStorageEngine] 默认管理员账户已存在";
    }

    m_initialized = true;
    qDebug() << "[UserStorageEngine] 数据库初始化完成:" << m_dbPath;
    return true;
}

bool UserStorageEngine::isInitialized() const
{
    QReadLocker locker(&m_lock);
    return m_initialized;
}

bool UserStorageEngine::createTables()
{
    QSqlQuery query(m_database);

    QString createTableSQL = R"(
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            permission INTEGER NOT NULL,
            created_at INTEGER NOT NULL,
            last_login INTEGER,
            active INTEGER NOT NULL DEFAULT 1
        )
    )";

    if (!query.exec(createTableSQL)) {
        m_lastError = QString("Failed to create users table: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool UserStorageEngine::createIndexes()
{
    QSqlQuery query(m_database);

    QString createUsernameIndex = "CREATE INDEX IF NOT EXISTS idx_username ON users(username)";
    if (!query.exec(createUsernameIndex)) {
        m_lastError = QString("Failed to create username index: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool UserStorageEngine::insertUser(const User& user)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare(R"(
        INSERT INTO users (username, password_hash, permission, created_at, last_login, active)
        VALUES (?, ?, ?, ?, ?, ?)
    )");

    query.addBindValue(user.username);
    query.addBindValue(user.passwordHash);
    query.addBindValue(static_cast<int>(user.permission));
    query.addBindValue(user.createdAt.toMSecsSinceEpoch());
    query.addBindValue(user.lastLogin.isValid() ? user.lastLogin.toMSecsSinceEpoch() : QVariant());
    query.addBindValue(user.active ? 1 : 0);

    if (!query.exec()) {
        m_lastError = QString("Failed to insert user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit userInserted(user);
    return true;
}

bool UserStorageEngine::updateUser(const User& user)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare(R"(
        UPDATE users SET
            username = ?,
            password_hash = ?,
            permission = ?,
            last_login = ?,
            active = ?
        WHERE id = ?
    )");

    query.addBindValue(user.username);
    query.addBindValue(user.passwordHash);
    query.addBindValue(static_cast<int>(user.permission));
    query.addBindValue(user.lastLogin.isValid() ? user.lastLogin.toMSecsSinceEpoch() : QVariant());
    query.addBindValue(user.active ? 1 : 0);
    query.addBindValue(user.id);

    if (!query.exec()) {
        m_lastError = QString("Failed to update user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit userUpdated(user);
    return true;
}

bool UserStorageEngine::deleteUser(int userId)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("DELETE FROM users WHERE id = ?");
    query.addBindValue(userId);

    if (!query.exec()) {
        m_lastError = QString("Failed to delete user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit userDeleted(userId);
    return true;
}

bool UserStorageEngine::deleteUser(const QString& username)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("DELETE FROM users WHERE username = ?");
    query.addBindValue(username);

    if (!query.exec()) {
        m_lastError = QString("Failed to delete user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    emit userDeleted(query.numRowsAffected());
    return true;
}

User UserStorageEngine::getUserById(int userId)
{
    QReadLocker locker(&m_lock);

    User user;

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return user;
    }

    QSqlQuery query(m_database);
    query.prepare("SELECT id, username, password_hash, permission, created_at, last_login, active FROM users WHERE id = ?");
    query.addBindValue(userId);

    if (!query.exec()) {
        m_lastError = QString("Failed to get user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return user;
    }

    if (query.next()) {
        user.id = query.value(0).toInt();
        user.username = query.value(1).toString();
        user.passwordHash = query.value(2).toString();
        user.permission = stringToPermission(query.value(3).toString());
        user.createdAt = QDateTime::fromMSecsSinceEpoch(query.value(4).toLongLong());
        if (!query.value(5).isNull()) {
            user.lastLogin = QDateTime::fromMSecsSinceEpoch(query.value(5).toLongLong());
        }
        user.active = query.value(6).toInt() == 1;
    }

    return user;
}

User UserStorageEngine::getUserByUsername(const QString& username)
{
    QReadLocker locker(&m_lock);

    User user;

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return user;
    }

    QSqlQuery query(m_database);
    query.prepare("SELECT id, username, password_hash, permission, created_at, last_login, active FROM users WHERE username = ?");
    query.addBindValue(username);

    if (!query.exec()) {
        m_lastError = QString("Failed to get user: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return user;
    }

    if (query.next()) {
        user.id = query.value(0).toInt();
        user.username = query.value(1).toString();
        user.passwordHash = query.value(2).toString();
        user.permission = stringToPermission(query.value(3).toString());
        user.createdAt = QDateTime::fromMSecsSinceEpoch(query.value(4).toLongLong());
        if (!query.value(5).isNull()) {
            user.lastLogin = QDateTime::fromMSecsSinceEpoch(query.value(5).toLongLong());
        }
        user.active = query.value(6).toInt() == 1;
    }

    return user;
}

QVector<User> UserStorageEngine::getAllUsers()
{
    QReadLocker locker(&m_lock);

    QVector<User> users;

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return users;
    }

    QSqlQuery query(m_database);
    query.prepare("SELECT id, username, password_hash, permission, created_at, last_login, active FROM users ORDER BY id");

    if (!query.exec()) {
        m_lastError = QString("Failed to get all users: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return users;
    }

    while (query.next()) {
        User user;
        user.id = query.value(0).toInt();
        user.username = query.value(1).toString();
        user.passwordHash = query.value(2).toString();
        user.permission = stringToPermission(query.value(3).toString());
        user.createdAt = QDateTime::fromMSecsSinceEpoch(query.value(4).toLongLong());
        if (!query.value(5).isNull()) {
            user.lastLogin = QDateTime::fromMSecsSinceEpoch(query.value(5).toLongLong());
        }
        user.active = query.value(6).toInt() == 1;
        users.append(user);
    }

    return users;
}

bool UserStorageEngine::updateLastLogin(int userId)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("UPDATE users SET last_login = ? WHERE id = ?");
    query.addBindValue(QDateTime::currentMSecsSinceEpoch());
    query.addBindValue(userId);

    if (!query.exec()) {
        m_lastError = QString("Failed to update last login: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool UserStorageEngine::changePassword(int userId, const QString& newPasswordHash)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("UPDATE users SET password_hash = ? WHERE id = ?");
    query.addBindValue(newPasswordHash);
    query.addBindValue(userId);

    if (!query.exec()) {
        m_lastError = QString("Failed to change password: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool UserStorageEngine::changePassword(const QString& username, const QString& newPasswordHash)
{
    QWriteLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("UPDATE users SET password_hash = ? WHERE username = ?");
    query.addBindValue(newPasswordHash);
    query.addBindValue(username);

    if (!query.exec()) {
        m_lastError = QString("Failed to change password: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    return true;
}

bool UserStorageEngine::userExists(const QString& username)
{
    QReadLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("SELECT COUNT(*) FROM users WHERE username = ?");
    query.addBindValue(username);

    if (!query.exec()) {
        m_lastError = QString("Failed to check user existence: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    if (query.next()) {
        return query.value(0).toInt() > 0;
    }

    return false;
}

bool UserStorageEngine::isUserActive(const QString& username)
{
    QReadLocker locker(&m_lock);

    if (!m_initialized || !m_database.isOpen()) {
        m_lastError = "Database not initialized";
        return false;
    }

    QSqlQuery query(m_database);
    query.prepare("SELECT active FROM users WHERE username = ?");
    query.addBindValue(username);

    if (!query.exec()) {
        m_lastError = QString("Failed to check user active status: %1").arg(query.lastError().text());
        emit errorOccurred(m_lastError);
        return false;
    }

    if (query.next()) {
        return query.value(0).toInt() == 1;
    }

    return false;
}

QString UserStorageEngine::getLastError() const
{
    QReadLocker locker(&m_lock);
    return m_lastError;
}

QString UserStorageEngine::permissionToString(UserPermission permission)
{
    switch (permission) {
        case UserPermission::VIEWER:
            return "VIEWER";
        case UserPermission::OPERATOR:
            return "OPERATOR";
        case UserPermission::ADMIN:
            return "ADMIN";
        default:
            return "VIEWER";
    }
}

UserPermission UserStorageEngine::stringToPermission(const QString& str)
{
    if (str == "VIEWER") {
        return UserPermission::VIEWER;
    } else if (str == "OPERATOR") {
        return UserPermission::OPERATOR;
    } else if (str == "ADMIN") {
        return UserPermission::ADMIN;
    }
    return UserPermission::VIEWER;
}

QString UserStorageEngine::hashPassword(const QString& password)
{
    QByteArray hash = QCryptographicHash::hash(
        password.toUtf8(),
        QCryptographicHash::Sha256
    );
    return hash.toHex();
}
