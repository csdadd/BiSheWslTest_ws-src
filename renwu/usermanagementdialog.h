#ifndef USERMANAGEMENTDIALOG_H
#define USERMANAGEMENTDIALOG_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include "userauthmanager.h"
#include "user.h"

class UserManagementDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UserManagementDialog(UserAuthManager* authManager, QWidget* parent = nullptr);
    ~UserManagementDialog();

private slots:
    void onAddUserClicked();
    void onDeleteUserClicked();
    void onChangePasswordClicked();
    void onRefreshClicked();
    void onCloseClicked();
    void onUserCreated(const User& user);
    void onUserDeleted(const QString& username);
    void onPermissionChanged(const QString& username, UserPermission newPermission);
    void onPasswordChanged();
    void onErrorOccurred(const QString& error);

private:
    void setupUI();
    void setupConnections();
    void loadUsers();
    void addUserToTable(const User& user);
    QString permissionToString(UserPermission permission);
    UserPermission stringToPermission(const QString& str);

private:
    UserAuthManager* m_authManager;

    QTableWidget* m_userTable;
    QPushButton* m_addUserButton;
    QPushButton* m_deleteUserButton;
    QPushButton* m_changePasswordButton;
    QPushButton* m_refreshButton;
    QPushButton* m_closeButton;
    QLabel* m_messageLabel;
};

#endif // USERMANAGEMENTDIALOG_H
