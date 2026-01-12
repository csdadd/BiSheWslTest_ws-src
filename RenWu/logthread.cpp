#include "logthread.h"
#include <QDir>
#include <QCoreApplication>
#include <QDebug>

LogThread::LogThread(QObject* parent)
    : BaseThread(parent)
    , m_maxFileSize(DEFAULT_MAX_FILE_SIZE)
    , m_maxFileCount(DEFAULT_MAX_FILE_COUNT)
{
    m_threadName = "LogThread";

    m_logDirectory = QCoreApplication::applicationDirPath() + "/logs";
    QDir dir;
    if (!dir.exists(m_logDirectory)) {
        dir.mkpath(m_logDirectory);
    }

    QString currentDate = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    m_logFilePath = m_logDirectory + QString("/robot_%1.log").arg(currentDate);
}

LogThread::~LogThread()
{
    stopThread();
}

void LogThread::setLogFilePath(const QString& path)
{
    QMutexLocker locker(&m_fileMutex);
    if (m_logFile.isOpen()) {
        m_logFile.close();
    }
    m_logFilePath = path;
    emit logFileChanged(path);
}

QString LogThread::getLogFilePath() const
{
    return m_logFilePath;
}

void LogThread::initialize()
{
    try {
        m_logFile.setFileName(m_logFilePath);
        if (m_logFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
            m_logStream.setDevice(&m_logFile);
            m_logStream.setCodec(QTextCodec::codecForName("UTF-8"));
        } else {
            emit threadError(QString("Failed to open log file: %1").arg(m_logFilePath));
        }

        m_storageEngine = new LogStorageEngine(nullptr);
        QString dbPath = m_logDirectory + "/app_logs.db";
        if (!m_storageEngine->initialize(dbPath)) {
            emit threadError(QString("Failed to initialize LogStorageEngine: %1").arg(m_storageEngine->getLastError()));
        }

        emit logFileChanged(m_logFilePath);
        emit logMessage("LogThread initialized successfully", LOG_INFO);

    } catch (const std::exception& e) {
        emit threadError(QString("Failed to initialize LogThread: %1").arg(e.what()));
        throw;
    }
}

void LogThread::process()
{
    // qDebug() << "[LogThread] 正在运行 - 处理日志队列";
    processLogQueue();
}

void LogThread::cleanup()
{
    processLogQueue();

    QMutexLocker locker(&m_fileMutex);
    if (m_logFile.isOpen()) {
        m_logStream.flush();
        m_logFile.close();
    }

    emit logMessage("LogThread cleanup completed", LOG_INFO);
}

void LogThread::writeLog(const QString& message, int level)
{
    LogEntry entry(message, level, QDateTime::currentDateTime(), "Application", "User");
    m_logQueue.enqueue(entry);
}

void LogThread::writeLogEntry(const LogEntry& entry)
{
    m_logQueue.enqueue(entry);
}

void LogThread::processLogQueue()
{
    QVector<StorageLogEntry> batchEntries;
    LogEntry entry;
    
    while (m_logQueue.tryDequeue(entry, 0)) {
        writeToFile(entry.message, entry.level, entry.timestamp, entry.source);
        
        StorageLogEntry storageEntry(
            entry.message,
            entry.level,
            entry.timestamp,
            entry.source,
            entry.category
        );
        batchEntries.append(storageEntry);
        
        if (batchEntries.size() >= 100) {
            if (m_storageEngine && m_storageEngine->isInitialized()) {
                m_storageEngine->insertLogs(batchEntries);
            }
            batchEntries.clear();
        }
    }
    
    if (!batchEntries.isEmpty() && m_storageEngine && m_storageEngine->isInitialized()) {
        m_storageEngine->insertLogs(batchEntries);
    }
}

void LogThread::writeToFile(const QString& message, int level, const QDateTime& timestamp, const QString& source)
{
    QMutexLocker locker(&m_fileMutex);

    if (!m_logFile.isOpen()) {
        return;
    }

    rotateLogFile();

    QString formattedMsg = formatLogMessage(message, level, timestamp, source);
    m_logStream << formattedMsg << Qt::endl;
    m_logStream.flush();
}

void LogThread::rotateLogFile()
{
    if (m_logFile.size() >= m_maxFileSize) {
        m_logFile.close();

        QFileInfo fileInfo(m_logFilePath);
        QString baseName = fileInfo.baseName();
        QString extension = fileInfo.completeSuffix();

        for (int i = m_maxFileCount - 1; i > 0; i--) {
            QString oldName = QString("%1/%2.%3.%4").arg(m_logDirectory, baseName).arg(i).arg(extension);
            QString newName = QString("%1/%2.%3.%4").arg(m_logDirectory, baseName).arg(i + 1).arg(extension);

            QDir dir;
            if (i == m_maxFileCount - 1) {
                dir.remove(newName);
            }
            dir.rename(oldName, newName);
        }

        QString backupName = QString("%1/%2.1.%3").arg(m_logDirectory, baseName, extension);
        QDir dir;
        dir.rename(m_logFilePath, backupName);

        m_logFile.open(QIODevice::WriteOnly | QIODevice::Append);
        m_logStream.setDevice(&m_logFile);
        m_logStream.setCodec(QTextCodec::codecForName("UTF-8"));
    }
}

QString LogThread::formatLogMessage(const QString& message, int level, const QDateTime& timestamp, const QString& source)
{
    QString timeStr = timestamp.toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString levelStr = levelToString(level);

    if (source.isEmpty()) {
        return QString("[%1] [%2] %3").arg(timeStr, levelStr, message);
    } else {
        return QString("[%1] [%2] [%3] %4").arg(timeStr, levelStr, source, message);
    }
}

QString LogThread::levelToString(int level)
{
    switch (level) {
        case LOG_DEBUG:
            return "DEBUG";
        case LOG_INFO:
            return "INFO";
        case LOG_WARNING:
            return "WARN";
        case LOG_ERROR:
            return "ERROR";
        case LOG_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

LogStorageEngine* LogThread::getStorageEngine() const
{
    return m_storageEngine;
}
