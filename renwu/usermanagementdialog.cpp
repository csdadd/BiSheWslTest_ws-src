#include "usermanagementdialog.h"
#include "ui_usermanagementdialog.h"
#include "changepassworddialog.h"
#include "adduserdialog.h"
#include <QDebug>
#include <QMessageBox>
#include <QBrush>
#include <QColor>

UserManagementDialog::UserManagementDialog(UserAuthManager* authManager, QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::UserManagementDialog)
    , m_authManager(authManager)
{
    ui->setupUi(this);
    resize(700, 500);
    ui->userTable->horizontalHeader()->setStretchLastSection(true);
    ui->messageLabel->hide();
    setupConnections();
    loadUsers();
}

UserManagementDialog::~UserManagementDialog()
{
    delete ui;
}

void UserManagementDialog::setupConnections()
{
    connect(ui->addUserButton, &QPushButton::clicked, this, &UserManagementDialog::onAddUserClicked);
    connect(ui->deleteUserButton, &QPushButton::clicked, this, &UserManagementDialog::onDeleteUserClicked);
    connect(ui->changePasswordButton, &QPushButton::clicked, this, &UserManagementDialog::onChangePasswordClicked);
    connect(ui->refreshButton, &QPushButton::clicked, this, &UserManagementDialog::onRefreshClicked);
    connect(ui->closeButton, &QPushButton::clicked, this, &UserManagementDialog::onCloseClicked);

    connect(m_authManager, &UserAuthManager::userCreated, this, &UserManagementDialog::onUserCreated);
    connect(m_authManager, &UserAuthManager::userDeleted, this, &UserManagementDialog::onUserDeleted);
    connect(m_authManager, &UserAuthManager::permissionChanged, this, &UserManagementDialog::onPermissionChanged);
    connect(m_authManager, &UserAuthManager::passwordChanged, this, &UserManagementDialog::onPasswordChanged);
    connect(m_authManager, &UserAuthManager::errorOccurred, this, &UserManagementDialog::onErrorOccurred);
}

void UserManagementDialog::loadUsers()
{
    ui->userTable->setRowCount(0);

    QVector<User> users = m_authManager->getAllUsers();

    for (const User& user : users) {
        addUserToTable(user);
    }

    qDebug() << "[UserManagementDialog] Loaded" << users.size() << "users";
}

void UserManagementDialog::addUserToTable(const User& user)
{
    int row = ui->userTable->rowCount();
    ui->userTable->insertRow(row);

    ui->userTable->setItem(row, 0, new QTableWidgetItem(QString::number(user.id)));
    ui->userTable->setItem(row, 1, new QTableWidgetItem(user.username));
    ui->userTable->setItem(row, 2, new QTableWidgetItem(permissionToString(user.permission)));
    ui->userTable->setItem(row, 3, new QTableWidgetItem(user.createdAt.toString("yyyy-MM-dd hh:mm:ss")));
    ui->userTable->setItem(row, 4, new QTableWidgetItem(user.active ? "启用" : "禁用"));

    if (!user.active) {
        for (int col = 0; col < ui->userTable->columnCount(); ++col) {
            if (ui->userTable->item(row, col)) {
                ui->userTable->item(row, col)->setForeground(QBrush(QColor(128, 128, 128)));
            }
        }
    }
}

void UserManagementDialog::onAddUserClicked()
{
    AddUserDialog dialog(m_authManager, this);
    if (dialog.exec() == QDialog::Accepted) {
        // 用户创建成功，loadUsers() 会通过信号自动刷新表格
    }
}

void UserManagementDialog::onDeleteUserClicked()
{
    int currentRow = ui->userTable->currentRow();

    if (currentRow < 0) {
        QMessageBox::warning(this, "警告", "请选择要删除的用户");
        return;
    }

    QString username = ui->userTable->item(currentRow, 1)->text();

    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "确认删除",
        QString("确定要删除用户 '%1' 吗？").arg(username),
        QMessageBox::Yes | QMessageBox::No
    );

    if (reply == QMessageBox::Yes) {
        if (m_authManager->deleteUser(username)) {
            QMessageBox::information(this, "成功", "用户删除成功");
        } else {
            QMessageBox::critical(this, "错误", m_authManager->getLastError());
        }
    }
}

void UserManagementDialog::onChangePasswordClicked()
{
    int currentRow = ui->userTable->currentRow();

    if (currentRow < 0) {
        QMessageBox::warning(this, "警告", "请选择要修改密码的用户");
        return;
    }

    QString username = ui->userTable->item(currentRow, 1)->text();

    ChangePasswordDialog dialog(ChangePasswordDialog::Mode::AdminReset, m_authManager, username, this);
    if (dialog.exec() == QDialog::Accepted) {
        // 密码修改成功，通过信号自动更新
    }
}

void UserManagementDialog::onRefreshClicked()
{
    loadUsers();
    ui->messageLabel->hide();
}

void UserManagementDialog::onCloseClicked()
{
    accept();
}

void UserManagementDialog::onUserCreated(const User& user)
{
    addUserToTable(user);
    ui->messageLabel->setText("用户创建成功");
    ui->messageLabel->setStyleSheet("color: green; margin: 5px;");
    ui->messageLabel->show();
    qDebug() << "[UserManagementDialog] User created:" << user.username;
}

void UserManagementDialog::onUserDeleted(const QString& username)
{
    for (int row = 0; row < ui->userTable->rowCount(); ++row) {
        if (ui->userTable->item(row, 1)->text() == username) {
            ui->userTable->removeRow(row);
            break;
        }
    }
    ui->messageLabel->setText("用户删除成功");
    ui->messageLabel->setStyleSheet("color: green; margin: 5px;");
    ui->messageLabel->show();
    qDebug() << "[UserManagementDialog] User deleted:" << username;
}

void UserManagementDialog::onPermissionChanged(const QString& username, UserPermission newPermission)
{
    for (int row = 0; row < ui->userTable->rowCount(); ++row) {
        if (ui->userTable->item(row, 1)->text() == username) {
            ui->userTable->item(row, 2)->setText(permissionToString(newPermission));
            break;
        }
    }
    ui->messageLabel->setText("权限修改成功");
    ui->messageLabel->setStyleSheet("color: green; margin: 5px;");
    ui->messageLabel->show();
    qDebug() << "[UserManagementDialog] Permission changed for user:" << username;
}

void UserManagementDialog::onPasswordChanged()
{
    ui->messageLabel->setText("密码修改成功");
    ui->messageLabel->setStyleSheet("color: green; margin: 5px;");
    ui->messageLabel->show();
    qDebug() << "[UserManagementDialog] Password changed";
}

void UserManagementDialog::onErrorOccurred(const QString& error)
{
    ui->messageLabel->setText(error);
    ui->messageLabel->setStyleSheet("color: red; margin: 5px;");
    ui->messageLabel->show();
    qDebug() << "[UserManagementDialog] Error:" << error;
}

QString UserManagementDialog::permissionToString(UserPermission permission)
{
    switch (permission) {
        case UserPermission::VIEWER:
            return "查看者";
        case UserPermission::OPERATOR:
            return "操作员";
        case UserPermission::ADMIN:
            return "管理员";
        default:
            return "未知";
    }
}

UserPermission UserManagementDialog::stringToPermission(const QString& str)
{
    if (str == "查看者") {
        return UserPermission::VIEWER;
    } else if (str == "操作员") {
        return UserPermission::OPERATOR;
    } else if (str == "管理员") {
        return UserPermission::ADMIN;
    }
    return UserPermission::VIEWER;
}
