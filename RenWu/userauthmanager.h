#ifndef USERAUTHMANAGER_H
#define USERAUTHMANAGER_H

#include <QObject>
#include <QString>
#include <QCryptographicHash>
#include <QDateTime>
#include "user.h"
#include "userstorageengine.h"

class UserAuthManager : public QObject
{
    Q_OBJECT

public:
    explicit UserAuthManager(QObject* parent = nullptr);
    ~UserAuthManager();

    bool initialize(UserStorageEngine* storageEngine);
    bool isInitialized() const;

    QString hashPassword(const QString& password);
    bool verifyPassword(const QString& password, const QString& passwordHash);

    bool login(const QString& username, const QString& password);
    bool logout();

    bool isLoggedIn() const;
    User getCurrentUser() const;
    QString getCurrentUsername() const;
    UserPermission getCurrentPermission() const;

    bool hasPermission(UserPermission requiredPermission) const;
    bool canView() const;
    bool canOperate() const;
    bool canAdmin() const;

    bool changePassword(const QString& oldPassword, const QString& newPassword);
    bool resetPassword(const QString& username, const QString& newPassword);

    bool createUser(const QString& username, const QString& password, UserPermission permission);
    bool deleteUser(const QString& username);
    bool updateUserPermission(const QString& username, UserPermission newPermission);

    QVector<User> getAllUsers();

    QString getLastError() const;

signals:
    void loginSuccess(const User& user);
    void loginFailed(const QString& reason);
    void logoutSuccess();
    void passwordChanged();
    void userCreated(const User& user);
    void userDeleted(const QString& username);
    void permissionChanged(const QString& username, UserPermission newPermission);
    void errorOccurred(const QString& error);

private:
    bool validatePassword(const QString& password);
    bool validateUsername(const QString& username);

private:
    UserStorageEngine* m_storageEngine;
    User m_currentUser;
    bool m_initialized;
    bool m_loggedIn;
    QString m_lastError;
    static constexpr int MIN_PASSWORD_LENGTH = 6;
    static constexpr int MAX_PASSWORD_LENGTH = 32;
    static constexpr int MIN_USERNAME_LENGTH = 3;
    static constexpr int MAX_USERNAME_LENGTH = 20;
};

#endif // USERAUTHMANAGER_H
