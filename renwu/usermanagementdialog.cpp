#include "usermanagementdialog.h"
#include <QDebug>
#include <QFormLayout>

UserManagementDialog::UserManagementDialog(UserAuthManager* authManager, QWidget* parent)
    : QDialog(parent)
    , m_authManager(authManager)
{
    setupUI();
    setupConnections();
    loadUsers();
}

UserManagementDialog::~UserManagementDialog()
{
}

void UserManagementDialog::setupUI()
{
    setWindowTitle("用户管理");
    setModal(true);
    resize(700, 500);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("用户管理", this);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; color: #333; margin-bottom: 10px;");
    mainLayout->addWidget(titleLabel);

    m_userTable = new QTableWidget(this);
    m_userTable->setColumnCount(5);
    m_userTable->setHorizontalHeaderLabels({"ID", "用户名", "权限", "创建时间", "状态"});
    m_userTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_userTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_userTable->horizontalHeader()->setStretchLastSection(true);
    m_userTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    mainLayout->addWidget(m_userTable);

    m_messageLabel = new QLabel(this);
    m_messageLabel->setAlignment(Qt::AlignCenter);
    m_messageLabel->setStyleSheet("color: red; margin: 5px;");
    m_messageLabel->hide();
    mainLayout->addWidget(m_messageLabel);

    QHBoxLayout* buttonLayout = new QHBoxLayout();

    m_addUserButton = new QPushButton("添加用户", this);
    m_addUserButton->setMinimumHeight(35);
    m_addUserButton->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; border-radius: 5px;");

    m_deleteUserButton = new QPushButton("删除用户", this);
    m_deleteUserButton->setMinimumHeight(35);
    m_deleteUserButton->setStyleSheet("background-color: #f44336; color: white; font-weight: bold; border-radius: 5px;");

    m_changePasswordButton = new QPushButton("修改密码", this);
    m_changePasswordButton->setMinimumHeight(35);
    m_changePasswordButton->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; border-radius: 5px;");

    m_refreshButton = new QPushButton("刷新", this);
    m_refreshButton->setMinimumHeight(35);
    m_refreshButton->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; border-radius: 5px;");

    m_closeButton = new QPushButton("关闭", this);
    m_closeButton->setMinimumHeight(35);
    m_closeButton->setStyleSheet("background-color: #9E9E9E; color: white; font-weight: bold; border-radius: 5px;");

    buttonLayout->addWidget(m_addUserButton);
    buttonLayout->addWidget(m_deleteUserButton);
    buttonLayout->addWidget(m_changePasswordButton);
    buttonLayout->addWidget(m_refreshButton);
    buttonLayout->addWidget(m_closeButton);

    mainLayout->addLayout(buttonLayout);
}

void UserManagementDialog::setupConnections()
{
    connect(m_addUserButton, &QPushButton::clicked, this, &UserManagementDialog::onAddUserClicked);
    connect(m_deleteUserButton, &QPushButton::clicked, this, &UserManagementDialog::onDeleteUserClicked);
    connect(m_changePasswordButton, &QPushButton::clicked, this, &UserManagementDialog::onChangePasswordClicked);
    connect(m_refreshButton, &QPushButton::clicked, this, &UserManagementDialog::onRefreshClicked);
    connect(m_closeButton, &QPushButton::clicked, this, &UserManagementDialog::onCloseClicked);

    connect(m_authManager, &UserAuthManager::userCreated, this, &UserManagementDialog::onUserCreated);
    connect(m_authManager, &UserAuthManager::userDeleted, this, &UserManagementDialog::onUserDeleted);
    connect(m_authManager, &UserAuthManager::permissionChanged, this, &UserManagementDialog::onPermissionChanged);
    connect(m_authManager, &UserAuthManager::passwordChanged, this, &UserManagementDialog::onPasswordChanged);
    connect(m_authManager, &UserAuthManager::errorOccurred, this, &UserManagementDialog::onErrorOccurred);
}

void UserManagementDialog::loadUsers()
{
    m_userTable->setRowCount(0);

    QVector<User> users = m_authManager->getAllUsers();

    for (const User& user : users) {
        addUserToTable(user);
    }

    qDebug() << "[UserManagementDialog] Loaded" << users.size() << "users";
}

void UserManagementDialog::addUserToTable(const User& user)
{
    int row = m_userTable->rowCount();
    m_userTable->insertRow(row);

    m_userTable->setItem(row, 0, new QTableWidgetItem(QString::number(user.id)));
    m_userTable->setItem(row, 1, new QTableWidgetItem(user.username));
    m_userTable->setItem(row, 2, new QTableWidgetItem(permissionToString(user.permission)));
    m_userTable->setItem(row, 3, new QTableWidgetItem(user.createdAt.toString("yyyy-MM-dd hh:mm:ss")));
    m_userTable->setItem(row, 4, new QTableWidgetItem(user.active ? "启用" : "禁用"));

    if (!user.active) {
        for (int col = 0; col < m_userTable->columnCount(); ++col) {
            if (m_userTable->item(row, col)) {
                m_userTable->item(row, col)->setForeground(QBrush(QColor(128, 128, 128)));
            }
        }
    }
}

void UserManagementDialog::onAddUserClicked()
{
    QDialog dialog(this);
    dialog.setWindowTitle("添加用户");
    dialog.setModal(true);
    dialog.setFixedSize(350, 300);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);

    QFormLayout* formLayout = new QFormLayout();

    QLineEdit* usernameEdit = new QLineEdit(&dialog);
    usernameEdit->setPlaceholderText("请输入用户名");
    formLayout->addRow("用户名:", usernameEdit);

    QLineEdit* passwordEdit = new QLineEdit(&dialog);
    passwordEdit->setPlaceholderText("请输入密码");
    passwordEdit->setEchoMode(QLineEdit::Password);
    formLayout->addRow("密码:", passwordEdit);

    QLineEdit* confirmPasswordEdit = new QLineEdit(&dialog);
    confirmPasswordEdit->setPlaceholderText("请确认密码");
    confirmPasswordEdit->setEchoMode(QLineEdit::Password);
    formLayout->addRow("确认密码:", confirmPasswordEdit);

    QComboBox* permissionCombo = new QComboBox(&dialog);
    permissionCombo->addItem("查看者 (VIEWER)", static_cast<int>(UserPermission::VIEWER));
    permissionCombo->addItem("操作员 (OPERATOR)", static_cast<int>(UserPermission::OPERATOR));
    permissionCombo->addItem("管理员 (ADMIN)", static_cast<int>(UserPermission::ADMIN));
    formLayout->addRow("权限:", permissionCombo);

    layout->addLayout(formLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* okButton = new QPushButton("确定", &dialog);
    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    layout->addLayout(buttonLayout);

    connect(okButton, &QPushButton::clicked, [&]() {
        QString username = usernameEdit->text().trimmed();
        QString password = passwordEdit->text();
        QString confirmPassword = confirmPasswordEdit->text();
        UserPermission permission = static_cast<UserPermission>(permissionCombo->currentData().toInt());

        if (username.isEmpty()) {
            QMessageBox::warning(&dialog, "错误", "请输入用户名");
            return;
        }

        if (password.isEmpty()) {
            QMessageBox::warning(&dialog, "错误", "请输入密码");
            return;
        }

        if (password != confirmPassword) {
            QMessageBox::warning(&dialog, "错误", "两次输入的密码不一致");
            return;
        }

        if (m_authManager->createUser(username, password, permission)) {
            QMessageBox::information(&dialog, "成功", "用户创建成功");
            dialog.accept();
        } else {
            QMessageBox::critical(&dialog, "错误", m_authManager->getLastError());
        }
    });

    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    dialog.exec();
}

void UserManagementDialog::onDeleteUserClicked()
{
    int currentRow = m_userTable->currentRow();

    if (currentRow < 0) {
        QMessageBox::warning(this, "警告", "请选择要删除的用户");
        return;
    }

    QString username = m_userTable->item(currentRow, 1)->text();

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
    int currentRow = m_userTable->currentRow();

    if (currentRow < 0) {
        QMessageBox::warning(this, "警告", "请选择要修改密码的用户");
        return;
    }

    QString username = m_userTable->item(currentRow, 1)->text();

    QDialog dialog(this);
    dialog.setWindowTitle("修改密码");
    dialog.setModal(true);
    dialog.setFixedSize(350, 250);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);

    QFormLayout* formLayout = new QFormLayout();

    QLineEdit* newPasswordEdit = new QLineEdit(&dialog);
    newPasswordEdit->setPlaceholderText("请输入新密码");
    newPasswordEdit->setEchoMode(QLineEdit::Password);
    formLayout->addRow("新密码:", newPasswordEdit);

    QLineEdit* confirmPasswordEdit = new QLineEdit(&dialog);
    confirmPasswordEdit->setPlaceholderText("请确认新密码");
    confirmPasswordEdit->setEchoMode(QLineEdit::Password);
    formLayout->addRow("确认密码:", confirmPasswordEdit);

    layout->addLayout(formLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* okButton = new QPushButton("确定", &dialog);
    QPushButton* cancelButton = new QPushButton("取消", &dialog);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    layout->addLayout(buttonLayout);

    connect(okButton, &QPushButton::clicked, [&]() {
        QString newPassword = newPasswordEdit->text();
        QString confirmPassword = confirmPasswordEdit->text();

        if (newPassword.isEmpty()) {
            QMessageBox::warning(&dialog, "错误", "请输入新密码");
            return;
        }

        if (newPassword != confirmPassword) {
            QMessageBox::warning(&dialog, "错误", "两次输入的密码不一致");
            return;
        }

        if (m_authManager->resetPassword(username, newPassword)) {
            QMessageBox::information(&dialog, "成功", "密码修改成功");
            dialog.accept();
        } else {
            QMessageBox::critical(&dialog, "错误", m_authManager->getLastError());
        }
    });

    connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    dialog.exec();
}

void UserManagementDialog::onRefreshClicked()
{
    loadUsers();
    m_messageLabel->hide();
}

void UserManagementDialog::onCloseClicked()
{
    accept();
}

void UserManagementDialog::onUserCreated(const User& user)
{
    addUserToTable(user);
    m_messageLabel->setText("用户创建成功");
    m_messageLabel->setStyleSheet("color: green; margin: 5px;");
    m_messageLabel->show();
    qDebug() << "[UserManagementDialog] User created:" << user.username;
}

void UserManagementDialog::onUserDeleted(const QString& username)
{
    for (int row = 0; row < m_userTable->rowCount(); ++row) {
        if (m_userTable->item(row, 1)->text() == username) {
            m_userTable->removeRow(row);
            break;
        }
    }
    m_messageLabel->setText("用户删除成功");
    m_messageLabel->setStyleSheet("color: green; margin: 5px;");
    m_messageLabel->show();
    qDebug() << "[UserManagementDialog] User deleted:" << username;
}

void UserManagementDialog::onPermissionChanged(const QString& username, UserPermission newPermission)
{
    for (int row = 0; row < m_userTable->rowCount(); ++row) {
        if (m_userTable->item(row, 1)->text() == username) {
            m_userTable->item(row, 2)->setText(permissionToString(newPermission));
            break;
        }
    }
    m_messageLabel->setText("权限修改成功");
    m_messageLabel->setStyleSheet("color: green; margin: 5px;");
    m_messageLabel->show();
    qDebug() << "[UserManagementDialog] Permission changed for user:" << username;
}

void UserManagementDialog::onPasswordChanged()
{
    m_messageLabel->setText("密码修改成功");
    m_messageLabel->setStyleSheet("color: green; margin: 5px;");
    m_messageLabel->show();
    qDebug() << "[UserManagementDialog] Password changed";
}

void UserManagementDialog::onErrorOccurred(const QString& error)
{
    m_messageLabel->setText(error);
    m_messageLabel->setStyleSheet("color: red; margin: 5px;");
    m_messageLabel->show();
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
