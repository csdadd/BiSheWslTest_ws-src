#ifndef USER_H
#define USER_H

#include <QString>
#include <QDateTime>

enum class UserPermission {
    VIEWER = 0,
    OPERATOR = 1,
    ADMIN = 2
};

struct User {
    int id;
    QString username;
    QString passwordHash;
    UserPermission permission;
    QDateTime createdAt;
    QDateTime lastLogin;
    bool active;

    User() : id(0), permission(UserPermission::VIEWER), active(true) {}
    User(int userId, const QString& name, const QString& hash, UserPermission perm)
        : id(userId), username(name), passwordHash(hash), permission(perm), active(true) {}
};

#endif // USER_H
