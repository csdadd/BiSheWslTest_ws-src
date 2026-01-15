#include "teststatusindicator.h"

void TestStatusIndicator::initTestCase()
{
}

void TestStatusIndicator::cleanupTestCase()
{
}

void TestStatusIndicator::init()
{
    m_manager = new StatusIndicatorManager();
}

void TestStatusIndicator::cleanup()
{
    delete m_manager;
    m_manager = nullptr;
}

void TestStatusIndicator::testIndicatorConstructor()
{
    StatusIndicator indicator;
    QCOMPARE(int(indicator.type), int(StatusType::Info));
    QVERIFY(indicator.message.isEmpty());
    QVERIFY(indicator.position.isNull());
}

void TestStatusIndicator::testIndicatorManagerConstructor()
{
    QVERIFY(m_manager != nullptr);
    QVERIFY(m_manager->getIndicators().isEmpty());
}

void TestStatusIndicator::testAddIndicator()
{
    StatusIndicator indicator;
    indicator.type = StatusType::Info;
    indicator.message = "Test message";
    indicator.position = QPointF(1.0, 2.0);
    indicator.timestamp = QDateTime::currentDateTime();

    m_manager->addIndicator(indicator);
    QCOMPARE(m_manager->getIndicators().size(), 1);
}

void TestStatusIndicator::testRemoveIndicator()
{
    StatusIndicator indicator;
    indicator.type = StatusType::Info;
    indicator.message = "Test message";
    indicator.position = QPointF(1.0, 2.0);
    indicator.timestamp = QDateTime::currentDateTime();

    m_manager->addIndicator(indicator);
    QCOMPARE(m_manager->getIndicators().size(), 1);

    m_manager->removeIndicator("Test message");
    QCOMPARE(m_manager->getIndicators().size(), 0);
}

void TestStatusIndicator::testGetIndicators()
{
    StatusIndicator indicator1;
    indicator1.type = StatusType::Info;
    indicator1.message = "Message 1";
    indicator1.position = QPointF(1.0, 2.0);
    indicator1.timestamp = QDateTime::currentDateTime();

    StatusIndicator indicator2;
    indicator2.type = StatusType::Warning;
    indicator2.message = "Message 2";
    indicator2.position = QPointF(3.0, 4.0);
    indicator2.timestamp = QDateTime::currentDateTime();

    m_manager->addIndicator(indicator1);
    m_manager->addIndicator(indicator2);

    QCOMPARE(m_manager->getIndicators().size(), 2);
}

void TestStatusIndicator::testClear()
{
    StatusIndicator indicator1;
    indicator1.type = StatusType::Info;
    indicator1.message = "Message 1";
    indicator1.position = QPointF(1.0, 2.0);
    indicator1.timestamp = QDateTime::currentDateTime();

    StatusIndicator indicator2;
    indicator2.type = StatusType::Warning;
    indicator2.message = "Message 2";
    indicator2.position = QPointF(3.0, 4.0);
    indicator2.timestamp = QDateTime::currentDateTime();

    m_manager->addIndicator(indicator1);
    m_manager->addIndicator(indicator2);
    QCOMPARE(m_manager->getIndicators().size(), 2);

    m_manager->clear();
    QCOMPARE(m_manager->getIndicators().size(), 0);
}

void TestStatusIndicator::testGetIndicatorsByType()
{
    StatusIndicator indicator1;
    indicator1.type = StatusType::Info;
    indicator1.message = "Info message";
    indicator1.position = QPointF(1.0, 2.0);
    indicator1.timestamp = QDateTime::currentDateTime();

    StatusIndicator indicator2;
    indicator2.type = StatusType::Warning;
    indicator2.message = "Warning message";
    indicator2.position = QPointF(3.0, 4.0);
    indicator2.timestamp = QDateTime::currentDateTime();

    StatusIndicator indicator3;
    indicator3.type = StatusType::Error;
    indicator3.message = "Error message";
    indicator3.position = QPointF(5.0, 6.0);
    indicator3.timestamp = QDateTime::currentDateTime();

    m_manager->addIndicator(indicator1);
    m_manager->addIndicator(indicator2);
    m_manager->addIndicator(indicator3);

    QList<StatusIndicator> infoIndicators = m_manager->getIndicatorsByType(StatusType::Info);
    QCOMPARE(infoIndicators.size(), 1);

    QList<StatusIndicator> warningIndicators = m_manager->getIndicatorsByType(StatusType::Warning);
    QCOMPARE(warningIndicators.size(), 1);

    QList<StatusIndicator> errorIndicators = m_manager->getIndicatorsByType(StatusType::Error);
    QCOMPARE(errorIndicators.size(), 1);
}

void TestStatusIndicator::testIndicatorInfoType()
{
    StatusIndicator indicator;
    indicator.type = StatusType::Info;
    QCOMPARE(int(indicator.type), int(StatusType::Info));
}

void TestStatusIndicator::testIndicatorWarningType()
{
    StatusIndicator indicator;
    indicator.type = StatusType::Warning;
    QCOMPARE(int(indicator.type), int(StatusType::Warning));
}

void TestStatusIndicator::testIndicatorErrorType()
{
    StatusIndicator indicator;
    indicator.type = StatusType::Error;
    QCOMPARE(int(indicator.type), int(StatusType::Error));
}

void TestStatusIndicator::testIndicatorPosition()
{
    StatusIndicator indicator;
    indicator.position = QPointF(1.0, 2.0);
    QCOMPARE(indicator.position.x(), 1.0);
    QCOMPARE(indicator.position.y(), 2.0);
}

void TestStatusIndicator::testIndicatorMessage()
{
    StatusIndicator indicator;
    indicator.message = "Test message";
    QCOMPARE(indicator.message, QString("Test message"));
}

void TestStatusIndicator::testIndicatorTimestamp()
{
    StatusIndicator indicator;
    QDateTime now = QDateTime::currentDateTime();
    indicator.timestamp = now;
    QCOMPARE(indicator.timestamp, now);
}
