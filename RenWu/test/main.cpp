#include <QtTest/QtTest>
#include "testmapconverter.h"
#include "testmapwidget.h"
#include "testintegration.h"
#include "testsystem.h"
#include "testmaploader.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    int result = 0;

    {
        TestMapConverter testMapConverter;
        result |= QTest::qExec(&testMapConverter, argc, argv);
    }

    {
        TestMapWidget testMapWidget;
        result |= QTest::qExec(&testMapWidget, argc, argv);
    }

    {
        TestIntegration testIntegration;
        result |= QTest::qExec(&testIntegration, argc, argv);
    }

    {
        TestSystem testSystem;
        result |= QTest::qExec(&testSystem, argc, argv);
    }

    {
        TestMapLoader testMapLoader;
        result |= QTest::qExec(&testMapLoader, argc, argv);
    }

    return result;
}
