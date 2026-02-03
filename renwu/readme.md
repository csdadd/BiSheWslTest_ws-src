这个项目只需要显示固定的地图图片和机器人的实时的位置，不需要订阅实时的地图

为测试方便，将开启登陆界面的代码，直接启动主界面

## 导航动作客户端类说明

本项目中有三个功能目的相同的导航动作客户端类：

### 1. NavStatusThread (navstatusthread.h/cpp)
**功能**: 订阅导航状态和路径话题
- 通过订阅 `/navigate_to_pose/_action/status` 话题获取导航状态
- 通过订阅 `/plan` 话题获取导航路径
- 使用 `SingleThreadedExecutor` 进行 `spin_some()`

**缺点**:
- 只能被动接收导航状态，无法主动发送导航目标
- 需要其他节点发送导航目标
- 测试表现不合适

### 2. NavigationActionClient (navigationactionclient.h/cpp)
**功能**: 导航动作客户端（非线程模式）
- 继承自 `QObject`
- 使用 `QTimer` 定时调用 `rclcpp::spin_some()`（3次重复调用）
- 可以发送导航目标、取消目标、获取反馈

**缺点**:
- 运行在主线程，可能阻塞界面
- 使用 QTimer + 多次 spin_some 的方式不够可靠
- 测试表现不合适

### 3. NavigationActionThread (navigationactionthread.h/cpp) **[推荐使用]**
**功能**: 导航动作客户端（独立线程模式）
- 继承自 `BaseThread`，运行在独立线程中
- 使用 `SingleThreadedExecutor` 的 `spin_once()` 实现可中断的阻塞式 spin
- 返回 `true` 给 `usesBlockingSpin()` 以启用阻塞式处理
- 可以发送导航目标、取消目标、获取反馈

**优点**:
- 独立线程运行，不会阻塞界面
- 使用 executor 的阻塞式 spin，更可靠
- 正确处理线程停止和清理
- 使用 lambda 捕获 this 设置回调

**状态**: 未测试，但设计最为合理

---

**推荐**: 使用 `NavigationActionThread` 进行导航功能开发。