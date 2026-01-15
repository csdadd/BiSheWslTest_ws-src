#include "userauthmanager.h"
#include <QDebug>

UserAuthManager::UserAuthManager(QObject* parent)
    : QObject(parent)
    , m_storageEngine(nullptr)
    , m_initialized(false)
    , m_loggedIn(false)
{
}

UserAuthManager::~UserAuthManager()
{
    logout();
}

bool UserAuthManager::initialize(UserStorageEngine* storageEngine)
{
    if (!storageEngine) {
        m_lastError = "Storage engine is null";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!storageEngine->isInitialized()) {
        m_lastError = "Storage engine not initialized";
        emit errorOccurred(m_lastError);
        return false;
    }

    m_storageEngine = storageEngine;
    m_initialized = true;
    qDebug() << "[UserAuthManager] Initialized successfully";
    return true;
}

bool UserAuthManager::isInitialized() const
{
    return m_initialized;
}

QString UserAuthManager::hashPassword(const QString& password)
{
    QByteArray hash = QCryptographicHash::hash(
        password.toUtf8(),
        QCryptographicHash::Sha256
    );
    return hash.toHex();
}

bool UserAuthManager::verifyPassword(const QString& password, const QString& passwordHash)
{
    QString hashedInput = hashPassword(password);
    return hashedInput == passwordHash;
}

bool UserAuthManager::login(const QString& username, const QString& password)
{
    if (!m_initialized) {
        m_lastError = "Auth manager not initialized";
        emit loginFailed(m_lastError);
        return false;
    }

    if (!validateUsername(username)) {
        m_lastError = "Invalid username format";
        emit loginFailed(m_lastError);
        return false;
    }

    if (!validatePassword(password)) {
        m_lastError = "Invalid password format";
        emit loginFailed(m_lastError);
        return false;
    }

    User user = m_storageEngine->getUserByUsername(username);

    if (user.id == 0) {
        m_lastError = "User not found";
        emit loginFailed(m_lastError);
        return false;
    }

    if (!user.active) {
        m_lastError = "User account is disabled";
        emit loginFailed(m_lastError);
        return false;
    }

    if (!verifyPassword(password, user.passwordHash)) {
        m_lastError = "Incorrect password";
        emit loginFailed(m_lastError);
        return false;
    }

    m_currentUser = user;
    m_loggedIn = true;

    m_storageEngine->updateLastLogin(user.id);

    qDebug() << "[UserAuthManager] User logged in:" << username;
    emit loginSuccess(m_currentUser);
    return true;
}

bool UserAuthManager::logout()
{
    if (!m_loggedIn) {
        return true;
    }

    QString username = m_currentUser.username;
    m_currentUser = User();
    m_loggedIn = false;

    qDebug() << "[UserAuthManager] User logged out:" << username;
    emit logoutSuccess();
    return true;
}

bool UserAuthManager::isLoggedIn() const
{
    return m_loggedIn;
}

User UserAuthManager::getCurrentUser() const
{
    return m_currentUser;
}

QString UserAuthManager::getCurrentUsername() const
{
    return m_currentUser.username;
}

UserPermission UserAuthManager::getCurrentPermission() const
{
    return m_currentUser.permission;
}

bool UserAuthManager::hasPermission(UserPermission requiredPermission) const
{
    if (!m_loggedIn) {
        return false;
    }
    return m_currentUser.permission >= requiredPermission;
}

bool UserAuthManager::canView() const
{
    return hasPermission(UserPermission::VIEWER);
}

bool UserAuthManager::canOperate() const
{
    return hasPermission(UserPermission::OPERATOR);
}

bool UserAuthManager::canAdmin() const
{
    return hasPermission(UserPermission::ADMIN);
}

bool UserAuthManager::changePassword(const QString& oldPassword, const QString& newPassword)
{
    if (!m_loggedIn) {
        m_lastError = "Not logged in";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!verifyPassword(oldPassword, m_currentUser.passwordHash)) {
        m_lastError = "Old password is incorrect";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!validatePassword(newPassword)) {
        m_lastError = "New password does not meet requirements";
        emit errorOccurred(m_lastError);
        return false;
    }

    QString newPasswordHash = hashPassword(newPassword);

    if (!m_storageEngine->changePassword(m_currentUser.id, newPasswordHash)) {
        m_lastError = m_storageEngine->getLastError();
        emit errorOccurred(m_lastError);
        return false;
    }

    m_currentUser.passwordHash = newPasswordHash;
    qDebug() << "[UserAuthManager] Password changed for user:" << m_currentUser.username;
    emit passwordChanged();
    return true;
}

bool UserAuthManager::resetPassword(const QString& username, const QString& newPassword)
{
    if (!canAdmin()) {
        m_lastError = "Insufficient permissions";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!validatePassword(newPassword)) {
        m_lastError = "New password does not meet requirements";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!m_storageEngine->userExists(username)) {
        m_lastError = "User not found";
        emit errorOccurred(m_lastError);
        return false;
    }

    QString newPasswordHash = hashPassword(newPassword);

    if (!m_storageEngine->changePassword(username, newPasswordHash)) {
        m_lastError = m_storageEngine->getLastError();
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[UserAuthManager] Password reset for user:" << username;
    emit passwordChanged();
    return true;
}

bool UserAuthManager::createUser(const QString& username, const QString& password, UserPermission permission)
{
    if (!canAdmin()) {
        m_lastError = "Insufficient permissions";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!validateUsername(username)) {
        m_lastError = "Invalid username format";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!validatePassword(password)) {
        m_lastError = "Invalid password format";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (m_storageEngine->userExists(username)) {
        m_lastError = "User already exists";
        emit errorOccurred(m_lastError);
        return false;
    }

    User newUser;
    newUser.username = username;
    newUser.passwordHash = hashPassword(password);
    newUser.permission = permission;
    newUser.createdAt = QDateTime::currentDateTime();
    newUser.active = true;

    if (!m_storageEngine->insertUser(newUser)) {
        m_lastError = m_storageEngine->getLastError();
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[UserAuthManager] User created:" << username;
    emit userCreated(newUser);
    return true;
}

bool UserAuthManager::deleteUser(const QString& username)
{
    if (!canAdmin()) {
        m_lastError = "Insufficient permissions";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (username == m_currentUser.username) {
        m_lastError = "Cannot delete your own account";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!m_storageEngine->userExists(username)) {
        m_lastError = "User not found";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (!m_storageEngine->deleteUser(username)) {
        m_lastError = m_storageEngine->getLastError();
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[UserAuthManager] User deleted:" << username;
    emit userDeleted(username);
    return true;
}

bool UserAuthManager::updateUserPermission(const QString& username, UserPermission newPermission)
{
    if (!canAdmin()) {
        m_lastError = "Insufficient permissions";
        emit errorOccurred(m_lastError);
        return false;
    }

    if (username == m_currentUser.username) {
        m_lastError = "Cannot change your own permission";
        emit errorOccurred(m_lastError);
        return false;
    }

    User user = m_storageEngine->getUserByUsername(username);

    if (user.id == 0) {
        m_lastError = "User not found";
        emit errorOccurred(m_lastError);
        return false;
    }

    user.permission = newPermission;

    if (!m_storageEngine->updateUser(user)) {
        m_lastError = m_storageEngine->getLastError();
        emit errorOccurred(m_lastError);
        return false;
    }

    qDebug() << "[UserAuthManager] Permission updated for user:" << username;
    emit permissionChanged(username, newPermission);
    return true;
}

QVector<User> UserAuthManager::getAllUsers()
{
    if (!canAdmin()) {
        m_lastError = "Insufficient permissions";
        emit errorOccurred(m_lastError);
        return QVector<User>();
    }

    return m_storageEngine->getAllUsers();
}

QString UserAuthManager::getLastError() const
{
    return m_lastError;
}

bool UserAuthManager::validatePassword(const QString& password)
{
    if (password.length() < MIN_PASSWORD_LENGTH) {
        return false;
    }

    if (password.length() > MAX_PASSWORD_LENGTH) {
        return false;
    }

    return true;
}

bool UserAuthManager::validateUsername(const QString& username)
{
    if (username.length() < MIN_USERNAME_LENGTH) {
        return false;
    }

    if (username.length() > MAX_USERNAME_LENGTH) {
        return false;
    }

    for (const QChar& c : username) {
        if (!c.isLetterOrNumber() && c != '_') {
            return false;
        }
    }

    return true;
}
