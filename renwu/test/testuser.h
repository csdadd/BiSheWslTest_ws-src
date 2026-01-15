#ifndef TESTUSER_H
#define TESTUSER_H

#include <QtTest/QtTest>
#include <QDateTime>
#include "user.h"

class TestUser : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void cleanupTestCase();
    void init();
    void cleanup();

    void testDefaultConstructor();
    void testParameterizedConstructor();
    void testPermissionEnumValues();
    void testUserFields();
    void testUserCopy();
};

#endif
