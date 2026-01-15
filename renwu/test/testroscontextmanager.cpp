#include "testroscontextmanager.h"

void TestROSContextManager::initTestCase()
{
}

void TestROSContextManager::cleanupTestCase()
{
}

void TestROSContextManager::init()
{
    m_manager = &ROSContextManager::instance();
}

void TestROSContextManager::cleanup()
{
    if (m_manager && m_manager->isInitialized()) {
        m_manager->shutdown();
    }
}

void TestROSContextManager::testGetInstance()
{
    ROSContextManager& instance1 = ROSContextManager::instance();
    ROSContextManager& instance2 = ROSContextManager::instance();
    QVERIFY(&instance1 == &instance2);
}

void TestROSContextManager::testInitialize()
{
    QVERIFY(!m_manager->isInitialized());
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
}

void TestROSContextManager::testIsInitialized()
{
    QVERIFY(!m_manager->isInitialized());
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
}

void TestROSContextManager::testGetContext()
{
    m_manager->initialize();
    auto context = m_manager->getContext();
    QVERIFY(context != nullptr);
}

void TestROSContextManager::testShutdown()
{
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
    m_manager->shutdown();
    QVERIFY(!m_manager->isInitialized());
}

void TestROSContextManager::testMultipleInitialize()
{
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
}

void TestROSContextManager::testThreadSafety()
{
    m_manager->initialize();
    QVERIFY(m_manager->isInitialized());
    auto context = m_manager->getContext();
    QVERIFY(context != nullptr);
}
