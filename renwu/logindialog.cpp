#include "logindialog.h"
#include "ui_logindialog.h"
#include <QDebug>

LoginDialog::LoginDialog(UserAuthManager* authManager, QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::LoginDialog)
    , m_authManager(authManager)
{
    ui->setupUi(this);
    setFixedSize(350, 250);
    ui->messageLabel->hide();
    setupConnections();
}

LoginDialog::~LoginDialog()
{
    delete ui;
}

User LoginDialog::getCurrentUser() const
{
    return m_currentUser;
}

void LoginDialog::setupConnections()
{
    connect(ui->loginButton, &QPushButton::clicked, this, &LoginDialog::onLoginClicked);
    connect(ui->cancelButton, &QPushButton::clicked, this, &LoginDialog::onCancelClicked);
    connect(m_authManager, &UserAuthManager::loginSuccess, this, &LoginDialog::onLoginSuccess);
    connect(m_authManager, &UserAuthManager::loginFailed, this, &LoginDialog::onLoginFailed);

    connect(ui->usernameEdit, &QLineEdit::returnPressed, this, &LoginDialog::onLoginClicked);
    connect(ui->passwordEdit, &QLineEdit::returnPressed, this, &LoginDialog::onLoginClicked);
}

void LoginDialog::onLoginClicked()
{
    QString username = ui->usernameEdit->text().trimmed();
    QString password = ui->passwordEdit->text();

    if (username.isEmpty()) {
        ui->messageLabel->setText("请输入用户名");
        ui->messageLabel->show();
        return;
    }

    if (password.isEmpty()) {
        ui->messageLabel->setText("请输入密码");
        ui->messageLabel->show();
        return;
    }

    ui->messageLabel->hide();
    ui->loginButton->setEnabled(false);
    ui->cancelButton->setEnabled(false);

    if (m_authManager->login(username, password)) {
        accept();
    } else {
        ui->loginButton->setEnabled(true);
        ui->cancelButton->setEnabled(true);
    }
}

void LoginDialog::onCancelClicked()
{
    reject();
}

void LoginDialog::onLoginSuccess(const User& user)
{
    m_currentUser = user;
    qDebug() << "[LoginDialog] Login successful for user:" << user.username;
}

void LoginDialog::onLoginFailed(const QString& reason)
{
    ui->messageLabel->setText(reason);
    ui->messageLabel->show();
    ui->loginButton->setEnabled(true);
    ui->cancelButton->setEnabled(true);
    qDebug() << "[LoginDialog] Login failed:" << reason;
}
