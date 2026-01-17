#ifndef CHANGEPASSWORDDIALOG_H
#define CHANGEPASSWORDDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include "userauthmanager.h"

namespace Ui {
class ChangePasswordDialog;
}

class ChangePasswordDialog : public QDialog
{
    Q_OBJECT

public:
    enum class Mode {
        SelfChange,      // 用户自己修改（需要旧密码）
        AdminReset       // 管理员重置（不需要旧密码）
    };

    explicit ChangePasswordDialog(Mode mode, UserAuthManager* authManager, const QString& username = "", QWidget* parent = nullptr);
    ~ChangePasswordDialog();

private slots:
    void onOkClicked();
    void onCancelClicked();

private:
    void setupUI();
    void setupConnections();

    Ui::ChangePasswordDialog* ui;
    Mode m_mode;
    UserAuthManager* m_authManager;
    QString m_username;
};

#endif // CHANGEPASSWORDDIALOG_H
