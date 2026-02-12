# Renwu 测试使用说明

## 概述

本目录包含 renwu 项目的所有单元测试。测试使用 Qt Test 框架编写，可通过命令行参数灵活控制运行哪些测试。

## 编译测试

```bash
cd /home/w/wheeltec_ros2
colcon build --packages-select renwu
source install/setup.bash
```

## 运行测试

### 列出所有可用测试

```bash
renwu_tests --list
# 或
renwu_tests -l
```

### 运行所有测试

```bash
renwu_tests -t all
# 或（默认就是 all）
renwu_tests
```

### 运行单个测试

```bash
# 运行历史日志模型测试
renwu_tests -t historylogmodel

# 运行日志存储引擎测试
renwu_tests -t logstorage

# 运行日志表格模型测试
renwu_tests -t logtable
```

### 运行多个指定测试

```bash
# 逗号分隔
renwu_tests -t historylogmodel,logtable,logfilter
```

### 使用分类快捷方式

| 快捷方式 | 说明 |
|---------|------|
| `log`   | 运行所有日志相关测试 |
| `map`   | 运行所有地图相关测试 |
| `nav`   | 运行所有导航相关测试 |
| `user`  | 运行所有用户相关测试 |
| `status` | 运行所有状态线程测试 |
| `base`  | 运行所有基础组件测试 |

```bash
# 运行所有日志相关测试
renwu_tests -t log

# 运行所有地图相关测试
renwu_tests -t map

# 运行所有导航相关测试
renwu_tests -t nav
```

### Qt Test 参数支持

```bash
# 详细输出
renwu_tests -t historylogmodel -v2

# 静默模式
renwu_tests -t historylogmodel -silent

# 列出测试函数
renwu_tests -t historylogmodel -functions
```

## 可用测试列表

### 日志相关 (log)
| 测试名称 | 说明 |
|---------|------|
| `logstorage` | 日志存储引擎测试 |
| `logtable` | 实时日志表格模型测试 |
| `logfilter` | 日志过滤代理模型测试 |
| `logquery` | 日志查询任务测试 |
| `logthread` | 日志线程测试 |
| `historylogmodel` | 历史日志表格模型测试 |

### 地图相关 (map)
| 测试名称 | 说明 |
|---------|------|
| `mapcache` | 地图缓存测试 |
| `mapconverter` | 地图转换器测试 |
| `mapwidget` | 地图控件测试 |
| `maploader` | 地图加载器测试 |
| `mapmarker` | 地图标记测试 |
| `mapthread` | 地图线程测试 |

### 导航相关 (nav)
| 测试名称 | 说明 |
|---------|------|
| `navactionclient` | 导航动作客户端测试 |
| `navactionthread` | 导航动作线程测试 |
| `pathvisualizer` | 路径可视化测试 |
| `navintegration` | 导航集成测试 |
| `nav2param` | Nav2 参数线程测试 |
| `nav2paramint` | Nav2 参数集成测试 |
| `nav2view` | Nav2 视图控件测试 |

### 用户相关 (user)
| 测试名称 | 说明 |
|---------|------|
| `user` | 用户类测试 |
| `userstorage` | 用户存储引擎测试 |
| `userauth` | 用户认证管理器测试 |
| `logindialog` | 登录对话框测试 |
| `usermgmt` | 用户管理对话框测试 |

### 状态线程 (status)
| 测试名称 | 说明 |
|---------|------|
| `robotstatus` | 机器人状态线程测试 |
| `navstatus` | 导航状态线程测试 |
| `sysmonitor` | 系统监控线程测试 |
| `statusindicator` | 状态指示器测试 |

### 基础组件 (base)
| 测试名称 | 说明 |
|---------|------|
| `basethread` | 基础线程测试 |
| `roscontext` | ROS 上下文管理器测试 |
| `threadsafequeue` | 线程安全队列测试 |

### 其他 (other)
| 测试名称 | 说明 |
|---------|------|
| `integration` | 集成测试 |
| `system` | 系统测试 |
| `mainwindow` | 主窗口测试 |

## 测试输出说明

```
将运行 2 个测试:
  - historylogmodel
  - logtable

********* 运行 historylogmodel *********
********* Start testing of TestHistoryLogTableModel *********
...
Totals: 22 passed, 0 failed, 0 skipped, 0 blacklisted, 6ms
********* Finished testing of TestHistoryLogTableModel *********
PASS: historylogmodel

==================== 测试总结 ====================
总计: 2, 通过: 1, 失败: 1
=================================================
```

- **passed**: 通过的测试数量
- **failed**: 失败的测试数量
- **skipped**: 跳过的测试数量
- **blacklisted**: 黑名单中的测试数量

## 添加新测试

1. 创建测试文件 `test/yourtest.h` 和 `test/yourtest.cpp`
2. 在 `CMakeLists.txt` 的 `TEST_SOURCES` 中添加新文件
3. 在 `test/main.cpp` 中：
   - 包含测试头文件 `#include "yourtest.h"`
   - 在 `allTests` 数组中添加测试描述：
     ```cpp
     {"yourtest", "category", RUN_TEST(YourTestClass)},
     ```

## 测试命名约定

- 测试文件: `test<name>.h` 和 `test<name>.cpp`
- 测试类名: `Test<Name>`
- 测试标识符: 小写，简洁描述

## 常见问题

**Q: 如何只运行历史日志相关的测试？**
```bash
renwu_tests -t historylogmodel
```

**Q: 如何查看详细的测试输出？**
```bash
renwu_tests -t historylogmodel -v2
```

**Q: 测试失败后如何调试？**
使用 `-v2` 参数获取详细输出，查看具体失败的测试行和错误信息。
