#include "logindialog.h"
#include <QDebug>

LoginDialog::LoginDialog(UserAuthManager* authManager, QWidget* parent)
    : QDialog(parent)
    , m_authManager(authManager)
{
    setupUI();
    setupConnections();
}

LoginDialog::~LoginDialog()
{
}

User LoginDialog::getCurrentUser() const
{
    return m_currentUser;
}

void LoginDialog::setupUI()
{
    setWindowTitle("登录");
    setModal(true);
    setFixedSize(350, 250);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("机器人导航监控系统", this);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; color: #333; margin-bottom: 10px;");
    mainLayout->addWidget(titleLabel);

    QFormLayout* formLayout = new QFormLayout();

    QLabel* usernameLabel = new QLabel("用户名:", this);
    m_usernameEdit = new QLineEdit(this);
    m_usernameEdit->setPlaceholderText("请输入用户名");
    formLayout->addRow(usernameLabel, m_usernameEdit);

    QLabel* passwordLabel = new QLabel("密码:", this);
    m_passwordEdit = new QLineEdit(this);
    m_passwordEdit->setPlaceholderText("请输入密码");
    m_passwordEdit->setEchoMode(QLineEdit::Password);
    formLayout->addRow(passwordLabel, m_passwordEdit);

    mainLayout->addLayout(formLayout);

    m_messageLabel = new QLabel(this);
    m_messageLabel->setAlignment(Qt::AlignCenter);
    m_messageLabel->setStyleSheet("color: red; margin: 5px;");
    m_messageLabel->hide();
    mainLayout->addWidget(m_messageLabel);

    QHBoxLayout* buttonLayout = new QHBoxLayout();

    m_loginButton = new QPushButton("登录", this);
    m_loginButton->setMinimumHeight(35);
    m_loginButton->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; border-radius: 5px;");

    m_cancelButton = new QPushButton("取消", this);
    m_cancelButton->setMinimumHeight(35);
    m_cancelButton->setStyleSheet("background-color: #f44336; color: white; font-weight: bold; border-radius: 5px;");

    buttonLayout->addWidget(m_loginButton);
    buttonLayout->addWidget(m_cancelButton);

    mainLayout->addLayout(buttonLayout);

    mainLayout->addStretch();
}

void LoginDialog::setupConnections()
{
    connect(m_loginButton, &QPushButton::clicked, this, &LoginDialog::onLoginClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &LoginDialog::onCancelClicked);
    connect(m_authManager, &UserAuthManager::loginSuccess, this, &LoginDialog::onLoginSuccess);
    connect(m_authManager, &UserAuthManager::loginFailed, this, &LoginDialog::onLoginFailed);

    connect(m_usernameEdit, &QLineEdit::returnPressed, this, &LoginDialog::onLoginClicked);
    connect(m_passwordEdit, &QLineEdit::returnPressed, this, &LoginDialog::onLoginClicked);
}

void LoginDialog::onLoginClicked()
{
    QString username = m_usernameEdit->text().trimmed();
    QString password = m_passwordEdit->text();

    if (username.isEmpty()) {
        m_messageLabel->setText("请输入用户名");
        m_messageLabel->show();
        return;
    }

    if (password.isEmpty()) {
        m_messageLabel->setText("请输入密码");
        m_messageLabel->show();
        return;
    }

    m_messageLabel->hide();
    m_loginButton->setEnabled(false);
    m_cancelButton->setEnabled(false);

    if (m_authManager->login(username, password)) {
        accept();
    } else {
        m_loginButton->setEnabled(true);
        m_cancelButton->setEnabled(true);
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
    m_messageLabel->setText(reason);
    m_messageLabel->show();
    m_loginButton->setEnabled(true);
    m_cancelButton->setEnabled(true);
    qDebug() << "[LoginDialog] Login failed:" << reason;
}
