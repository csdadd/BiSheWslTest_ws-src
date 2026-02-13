# Renwu 模块规范文档

> 本文档记录各模块已确定的实现特性，作为后续开发的参考基准。已锁定的模块不再修改核心设计。

---

## 目录

1. [实时日志模块](#1-实时日志模块) *(已锁定)*

---

## 1. 实时日志模块

**状态**: 已锁定
**锁定日期**: 2026-02-13
**相关文件**: `loglevel.h`, `logutils.h`, `threadsafequeue.h`, `logthread.h/cpp`, `logstorageengine.h/cpp`, `logtablemodel.h/cpp`, `logfilterproxymodel.h/cpp`, `historylogmodel.h/cpp`, `logquerytask.h/cpp`, `systemmonitorthread.h/cpp`

### 1.1 日志级别定义

```cpp
enum class LogLevel : int {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    FATAL = 4,
    HIGHFREQ = 5,   // 高频日志
    ODOMETRY = 6    // 里程计日志
};
```

| 级别 | 数值 | 用途 | 显示颜色 (RGB) |
|------|------|------|----------------|
| DEBUG | 0 | 调试信息 | (128, 128, 128) 灰色 |
| INFO | 1 | 常规信息 | (0, 0, 0) 黑色 |
| WARNING | 2 | 警告 | (255, 165, 0) 橙色 |
| ERROR | 3 | 错误 | (255, 0, 0) 红色 |
| FATAL | 4 | 致命错误 | (139, 0, 0) 深红色 |
| HIGHFREQ | 5 | 高频日志 | (100, 100, 255) 蓝色 |
| ODOMETRY | 6 | 里程计数据 | 不显示 |

---

### 1.2 日志容器大小

| 容器 | 位置 | 大小限制 | 用途 |
|------|------|----------|------|
| `m_allLogs` | `mainwindow.h` | 10000 条 | 主窗口内存缓存 |
| `m_logs` | `logtablemodel.h` | 1000 条 | UI 表格显示 |
| `BATCH_THRESHOLD` | `logtablemodel.h` | 100 条 | UI 批量更新阈值 |
| `m_batchUpdateInterval` | `logtablemodel.h` | 200 ms | UI 批量更新间隔 |
| `m_logQueue` | `logthread.h` | 无限制 | 线程安全队列 |
| `BATCH_WRITE_INTERVAL_MS` | `logthread.h` | 2000 ms | 文件/数据库批量写入间隔 |
| `BATCH_INSERT_SIZE` | `logstorageengine.h` | 100 条 | 数据库批量插入大小 |
| `m_pageSize` | `historylogmodel.h` | 100 条 | 历史日志分页大小 |

---

### 1.3 日志数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                         日志产生源                               │
├─────────────────────────────────────────────────────────────────┤
│  SystemMonitorThread:                                           │
│    - /rosout_agg (rcl_interfaces::msg::Log)                     │
│    - /mobile_base/sensors/bumper_pointcloud (碰撞检测)           │
│    - /behavior_tree_log (行为树日志)                             │
│                                                                 │
│  其他线程: RobotStatusThread, NavStatusThread 等                 │
│    - 通过 emit logMessage() 信号产生日志                         │
└──────────────────────────┬──────────────────────────────────────┘
                           │ emit logMessageReceived()
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                  MainWindow::onLogMessageReceived()              │
├─────────────────────────────────────────────────────────────────┤
│  addLogEntry(entry) 分发至三处:                                  │
│                                                                 │
│  1. m_logThread->writeLogEntry(entry)  → 持久化存储              │
│  2. m_allLogs.append(entry)            → 内存缓存 (10000 条)     │
│  3. m_logTableModel->addLogEntry(entry) → UI 显示 (1000 条)      │
│                                                                 │
│  特殊处理: ODOMETRY 级别不进入 UI，直接存储到独立表                │
└──────────────────────────┬──────────────────────────────────────┘
                           │
         ┌─────────────────┴─────────────────┐
         ▼                                   ▼
┌─────────────────────────┐   ┌─────────────────────────────────────┐
│       LogThread         │   │           LogTableModel              │
├─────────────────────────┤   ├─────────────────────────────────────┤
│ ThreadSafeQueue         │   │ m_pendingLogs (批量缓冲)             │
│         ↓               │   │         ↓                            │
│ processLogQueue()       │   │ flushPendingLogs()                   │
│         ↓               │   │ (200ms 或 100 条触发)                 │
│ 按级别分流:              │   │         ↓                            │
│  • HIGHFREQ → 高频表    │   │ m_logs (最大 1000 条)                │
│  • ODOMETRY → 里程计表  │   └─────────────────────────────────────┘
│  • 其他 → 日志表+文件   │
└─────────────────────────┘
```

---

### 1.4 日志级别处理差异

| 级别类型 | UI 显示 | 写入文件 | 写入数据库表 | 批量写入策略 |
|----------|---------|----------|--------------|--------------|
| 普通日志 (DEBUG/INFO/WARNING/ERROR/FATAL) | 是 | 是 | `logs` | 2 秒定时批量 |
| HIGHFREQ | 是 | 否 | `high_freq_logs` | 4 条立即批量 |
| ODOMETRY | 否 | 否 | `odometry_logs` | 100 条批量 |

---

### 1.5 三级缓冲架构

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   第一级缓冲     │ ──▶ │   第二级缓冲     │ ──▶ │   第三级缓冲     │
│ ThreadSafeQueue │     │    m_allLogs    │     │  m_logs (UI)    │
│   (线程间传输)   │     │  (内存缓存)      │     │   (界面显示)    │
│    无限制       │     │   10000 条      │     │    1000 条      │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

**设计目的**:
- 第一级: 解耦日志产生与处理线程，保证线程安全
- 第二级: 内存缓存全量日志，支持历史查询
- 第三级: 限制 UI 显示数量，保证界面性能

---

### 1.6 批量写入优化

| 类型 | 触发条件 | 目标 |
|------|----------|------|
| UI 更新 | 200ms 定时 或 累计 100 条 | 减少 UI 重绘次数 |
| 普通日志持久化 | 2 秒定时 | 减少 I/O 操作 |
| HIGHFREQ 日志 | 累计 4 条 | 高频数据快速落盘 |
| ODOMETRY 日志 | 累计 100 条 | 批量插入优化 |
| 数据库插入 | 累计 100 条 | 事务批量插入 |

---

### 1.7 特殊级别隔离

#### HIGHFREQ (高频日志)
- **用途**: 记录高频传感器数据或状态变化
- **特点**: 独立表存储，不写文件，4 条批量写入
- **显示**: 在 UI 中显示，蓝色标识

#### ODOMETRY (里程计日志)
- **用途**: 记录里程计原始数据
- **特点**: 独立表存储，不写文件，不在 UI 显示
- **原因**: 数据量大，避免刷爆界面和日志文件

---

### 1.8 日志轮转

| 参数 | 值 | 说明 |
|------|-----|------|
| `DEFAULT_MAX_FILE_SIZE` | 10 MB | 单个日志文件最大大小 |
| `DEFAULT_MAX_FILE_COUNT` | 5 | 保留历史文件数量 |
| 文件命名 | `robot_YYYY-MM-DD.log` | 按日期命名 |
| 轮转触发 | 文件大小超过 10MB | 自动创建新文件 |

**轮转策略**:
1. 当前文件超过 10MB 时，自动创建新文件
2. 最多保留 5 个历史文件
3. 日期变更时自动切换新文件

---

### 1.9 数据库表结构

#### logs (普通日志表)
```sql
CREATE TABLE logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp INTEGER NOT NULL,
    level INTEGER NOT NULL,
    message TEXT NOT NULL,
    source TEXT,
    category TEXT,
    file_path TEXT,
    line_number INTEGER,
    created_at INTEGER DEFAULT (strftime('%s', 'now'))
);

-- 索引
CREATE INDEX idx_logs_timestamp ON logs(timestamp);
CREATE INDEX idx_logs_level ON logs(level);
CREATE INDEX idx_logs_level_timestamp ON logs(level, timestamp);
```

#### high_freq_logs (高频日志表)
```sql
-- 结构与 logs 表相同
CREATE TABLE high_freq_logs (...);
```

#### odometry_logs (里程计表)
```sql
-- 结构与 logs 表相同
CREATE TABLE odometry_logs (...);
```

---

### 1.10 存储路径

| 存储类型 | 路径 |
|----------|------|
| 日志文件 | `<应用目录>/logs/robot_YYYY-MM-DD.log` |
| 数据库 | `<AppDataLocation>/logs.db` |

---

## 2. [预留模块]

<!-- 模板: 复制以下结构添加新模块

### 2.X 模块名称

**状态**: 已锁定 / 开发中
**锁定日期**: YYYY-MM-DD
**相关文件**: file1.h, file2.cpp, ...

#### 2.X.1 功能描述
[描述模块的核心功能]

#### 2.X.2 关键设计
[描述关键设计决策]

#### 2.X.3 接口定义
[描述对外接口]

-->

---

## 变更记录

| 日期 | 模块 | 变更内容 |
|------|------|----------|
| 2026-02-13 | 实时日志 | 初始锁定，记录所有已确定特性 |
