#include "testlogindialog.h"

void TestLoginDialog::initTestCase()
{
    QTemporaryFile tempFile;
    tempFile.open();
    m_testDbPath = tempFile.fileName() + ".db";
    tempFile.close();
}

void TestLoginDialog::cleanupTestCase()
{
    QFile::remove(m_testDbPath);
}

void TestLoginDialog::init()
{
    m_storageEngine = new UserStorageEngine();
    m_storageEngine->initialize(m_testDbPath);

    m_authManager = new UserAuthManager();
    m_authManager->initialize(m_storageEngine);

    m_dialog = new LoginDialog(m_authManager);
}

void TestLoginDialog::cleanup()
{
    delete m_dialog;
    m_dialog = nullptr;
    delete m_authManager;
    m_authManager = nullptr;
    delete m_storageEngine;
    m_storageEngine = nullptr;
    QFile::remove(m_testDbPath);
}

void TestLoginDialog::testConstructor()
{
    QVERIFY(m_dialog != nullptr);
    QVERIFY(m_dialog->windowTitle().contains("登录", Qt::CaseInsensitive));
}

void TestLoginDialog::testGetCurrentUser()
{
    User user = m_dialog->getCurrentUser();
    QVERIFY(user.getId() == 0);
}

void TestLoginDialog::testOnLoginClicked()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");

    if (usernameEdit && passwordEdit) {
        usernameEdit->setText("testuser");
        passwordEdit->setText("password123");

        QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
        if (loginButton) {
            loginButton->click();
            QTest::qWait(100);
        }
    }
}

void TestLoginDialog::testOnLoginClickedWithValidCredentials()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");

    if (usernameEdit && passwordEdit) {
        usernameEdit->setText("testuser");
        passwordEdit->setText("password123");

        QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
        if (loginButton) {
            QSignalSpy spy(m_dialog, &QDialog::accepted);
            loginButton->click();
            QTest::qWait(100);
        }
    }
}

void TestLoginDialog::testOnLoginClickedWithInvalidCredentials()
{
    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");

    if (usernameEdit && passwordEdit) {
        usernameEdit->setText("wronguser");
        passwordEdit->setText("wrongpassword");

        QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
        if (loginButton) {
            loginButton->click();
            QTest::qWait(100);
        }
    }
}

void TestLoginDialog::testOnCancelClicked()
{
    QPushButton* cancelButton = m_dialog->findChild<QPushButton*>("cancelButton");
    if (cancelButton) {
        QSignalSpy spy(m_dialog, &QDialog::rejected);
        cancelButton->click();
        QTest::qWait(100);
    }
}

void TestLoginDialog::testOnLoginSuccess()
{
    m_authManager->createUser("testuser", "password123", UserPermission::ADMIN);

    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");

    if (usernameEdit && passwordEdit) {
        usernameEdit->setText("testuser");
        passwordEdit->setText("password123");

        QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
        if (loginButton) {
            loginButton->click();
            QTest::qWait(100);

            User user = m_dialog->getCurrentUser();
            QVERIFY(user.getUsername() == "testuser");
        }
    }
}

void TestLoginDialog::testOnLoginFailed()
{
    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");

    if (usernameEdit && passwordEdit) {
        usernameEdit->setText("wronguser");
        passwordEdit->setText("wrongpassword");

        QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
        if (loginButton) {
            loginButton->click();
            QTest::qWait(100);

            User user = m_dialog->getCurrentUser();
            QVERIFY(user.getId() == 0);
        }
    }
}

void TestLoginDialog::testUsernameInput()
{
    QLineEdit* usernameEdit = m_dialog->findChild<QLineEdit*>("usernameEdit");
    QVERIFY(usernameEdit != nullptr);
}

void TestLoginDialog::testPasswordInput()
{
    QLineEdit* passwordEdit = m_dialog->findChild<QLineEdit*>("passwordEdit");
    QVERIFY(passwordEdit != nullptr);
    QVERIFY(passwordEdit->echoMode() == QLineEdit::Password);
}

void TestLoginDialog::testLoginButton()
{
    QPushButton* loginButton = m_dialog->findChild<QPushButton*>("loginButton");
    QVERIFY(loginButton != nullptr);
}

void TestLoginDialog::testCancelButton()
{
    QPushButton* cancelButton = m_dialog->findChild<QPushButton*>("cancelButton");
    QVERIFY(cancelButton != nullptr);
}
