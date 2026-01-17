#ifndef LOGINDIALOG_H
#define LOGINDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include "userauthmanager.h"

namespace Ui {
class LoginDialog;
}

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
    void setupConnections();

    Ui::LoginDialog* ui;
    UserAuthManager* m_authManager;
    User m_currentUser;
};

#endif // LOGINDIALOG_H
