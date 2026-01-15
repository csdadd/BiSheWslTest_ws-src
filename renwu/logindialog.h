#ifndef LOGINDIALOG_H
#define LOGINDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include "userauthmanager.h"

class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LoginDialog(UserAuthManager* authManager, QWidget* parent = nullptr);
    ~LoginDialog();

    User getCurrentUser() const;

private slots:
    void onLoginClicked();
    void onCancelClicked();
    void onLoginSuccess(const User& user);
    void onLoginFailed(const QString& reason);

private:
    void setupUI();
    void setupConnections();

private:
    UserAuthManager* m_authManager;
    User m_currentUser;

    QLineEdit* m_usernameEdit;
    QLineEdit* m_passwordEdit;
    QPushButton* m_loginButton;
    QPushButton* m_cancelButton;
    QLabel* m_messageLabel;
};

#endif // LOGINDIALOG_H
