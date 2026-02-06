这个项目只需要显示固定的地图图片和机器人的实时的位置，不需要订阅实时的地图

为测试方便，将开启登陆界面的代码，直接启动主界面（已测试完毕，恢复了登录功能）

## 废弃代码说明

以下代码文件已废弃，仅作为工作量证明保留：

| 文件 | 说明 |
|------|------|
| `navstatusthread.h/cpp` | 导航状态订阅线程，订阅 `/navigate_to_pose/_action/status` 和 `/plan` |
| `navigationactionclient.h/cpp` | 导航动作客户端（非线程模式），使用 QTimer + spin_some |
| `mapthread.h/cpp` | 实时地图订阅线程，订阅 `/map` 话题 |
| `mapwidget.h/cpp` | 基础地图显示组件，使用 QGraphicsView 架构 |

**保留原因**: 证明开发工作量

| 废弃的代码文件 | 废弃的测试文件 | 废弃原因 |
|---------------|---------------|---------|
| `navstatusthread.h/cpp` | `test/testnavstatusthread.h/cpp` | 已被 NavigationActionThread 替代 |
| `navigationactionclient.h/cpp` | `test/testnavigationactionclient.h/cpp` | 已被 NavigationActionThread（独立线程模式）替代 |
| `mapthread.h/cpp` | `test/testmapthread.h/cpp` | 项目改用固定地图图片，不再订阅实时地图 |
| `mapwidget.h/cpp` | `test/testmapwidget.h/cpp` | 已被 Nav2ViewWidget（高级导航可视化组件）替代 |

**测试跳过方式**: 在各测试类的 `initTestCase()` 中使用 `QSKIP()` 跳过整个测试套件

---

## 当前使用的代码

| 文件 | 说明 |
|------|------|
| `navigationactionthread.h/cpp` | 导航动作客户端（独立线程模式），**推荐使用** |
| `nav2viewwidget.h/cpp` | 高级导航可视化组件，显示激光点云、路径、位姿 |
| `robotstatusthread.h/cpp` | 机器人状态订阅线程（电池、位姿、里程计等） |
| `systemmonitorthread.h/cpp` | 系统监控线程（日志、碰撞检测、行为树日志） |

---

## 导航动作客户端演进

项目经历了三个版本的导航动作客户端实现：

1. **NavStatusThread** → 只能被动接收状态，无法主动发送目标
2. **NavigationActionClient** → 非线程模式，可能阻塞界面
3. **NavigationActionThread** → 独立线程，阻塞式 spin，**最终采用**

## 地图显示组件演进

1. **MapThread + MapWidget** → 订阅实时地图，**已废弃**
2. **Nav2ViewWidget** → 固定地图图片 + 实时位姿显示，**最终采用**

---

## Debug日志注释说明

以下debug日志输出已被注释（减少控制台噪音）：

| 文件 | 被注释的日志 |
|------|-------------|
| `navstatusthread.cpp:67` | `[NavStatusThread] 正在运行 - 获取导航状态、反馈和路径信息` |
| `robotstatusthread.cpp:94` | `[RobotStatusThread] 正在运行 - 获取电池、位置、里程计和诊断信息` |
| `systemmonitorthread.cpp:84` | `[SystemMonitorThread] 正在运行 - 监控ROS日志、碰撞检测和行为树` |
| `logthread.cpp:73` | `[LogThread] 正在运行 - 处理日志队列` |

**注释原因**: 已测试通过，且这些debug日志每100次循环输出一次，产生大量重复信息，影响控制台可读性。


