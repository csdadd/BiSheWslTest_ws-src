# ThreePointTurn 三点掉头恢复行为使用说明

## 功能概述

`ThreePointTurn` 是为Nav2导航框架开发的自定义恢复行为节点，专门用于阿克曼转向的小型机器人在狭窄空间内进行180°掉头。

### 适用场景

- 狭窄走廊或通道中的掉头
- 死胡同处的转向
- 导航失败后的恢复行为
- 无法原地旋转的阿克曼车型的转向操作

### 适用车型

该恢复行为仅支持阿克曼转向车型：
- `mini_akm` - 小型阿克曼机器人
- `senior_akm` - 大型阿克曼机器人
- `top_akm_bs` - 顶系列阿克曼机器人（带悬挂）
- `top_akm_dl` - 顶系列阿克曼机器人（不带悬挂）

**不支持**：差速驱动、麦克纳姆轮、全向轮等可原地旋转的车型（这些车型应使用Spin恢复行为）。

---

## 三点掉头原理

三点掉头是一种经典的汽车驾驶技巧，通过三个阶段的运动实现180°转向：

```
初始位置 → → → →    (Phase 1: 直线前进)
           ↘
            ↘        (Phase 2: 右转后退)
             ↘
              ●
             ↗
            ↗        (Phase 3: 左转前进)
           ↗
← ← ← ← ←          (完成180°掉头)
```

### 阶段说明

| 阶段 | 动作 | 转向角 | 速度 | 结束条件 |
|------|------|--------|------|----------|
| Phase 1 | 直线前进 | 0° | 正向 | 行驶距离 ≥ forward_dist |
| Phase 2 | 右转后退 | -70%最大转向角 | 负向 | 朝向变化 ≥ 90° |
| Phase 3 | 左转前进 | +70%最大转向角 | 正向 | 朝向变化 ≥ 90° |

---

## 在Nav2中的集成

### 1. 导航参数配置

在阿克曼车型的导航参数文件中（如 `param_mini_akm.yaml`），ThreePointTurn已被自动集成：

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "three_point_turn"]
    three_point_turn:
      plugin: "nav2_behaviors/ThreePointTurn"
```

### 2. 行为树配置

在默认行为树 `navigate_to_pose_w_replanning_and_recovery.xml` 中：

```xml
<RoundRobin name="RecoveryActions">
  <Sequence name="ClearingActions">
    <ClearEntireCostmap name="ClearLocalCostmap-Subtree" .../>
    <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" .../>
  </Sequence>
  <ThreePointTurn forward_dist="0.75" backward_dist="1.0"
                 max_speed="0.5" max_steering_angle="0.5"/>
  <Spin spin_dist="1.57"/>
  <Wait wait_duration="5"/>
  <BackUp backup_dist="0.15" backup_speed="0.025"/>
</RoundRobin>
```

### 3. 参数文件

ThreePointTurn的参数配置文件位于：
```
src/wheeltec_robot_nav2/param/three_point_turn_recoveries.yaml
```

---

## 使用方法

### 启动导航系统

```bash
# 1. 启动机器人底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 2. 启动导航（包含ThreePointTurn恢复行为）
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py
```

### 通过行为树使用

ThreePointTurn作为恢复行为，会在导航失败时自动触发。无需手动调用。

### 通过Action直接调用

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ThreePointTurn

def send_three_point_turn_goal():
    node = rclpy.create_node('three_point_turn_client')
    action_client = ActionClient(node, ThreePointTurn, 'ThreePointTurn')

    goal = ThreePointTurn.Goal()
    goal.forward_dist = 0.75
    goal.backward_dist = 1.0
    goal.max_speed = 0.5
    goal.max_steering_angle = 0.5
    goal.time_allowance.sec = 10

    action_client.wait_for_server()
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
```

---

## 话题接口

### 发布的话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/ackermann_cmd` | `ackermann_msgs::msg::AckermannDrive` | 阿克曼驱动指令 |

### 订阅的话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/local_costmap/costmap_raw` | `nav2_msgs::msg::Costmap` | 局部代价地图 |
| `/local_costmap/published_footprint` | `geometry_msgs::msg::PolygonStamped` | 机器人足迹 |

---

## 行为树端口

### 输入端口

| 端口名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `forward_dist` | float | 0.75 | 前进距离（米） |
| `backward_dist` | float | 1.0 | 后退距离（米） |
| `max_speed` | float | 0.5 | 最大速度（m/s） |
| `max_steering_angle` | float | 0.5 | 最大转向角（弧度） |

---

## 反馈信息

ThreePointTurn执行时会实时发布反馈：

```cpp
// Feedback 内容
current_phase          // 当前阶段（1/2/3）
distance_traveled      // 已行驶距离（米）
current_yaw            // 当前朝向角变化（弧度）
current_steering_angle // 当前转向角（弧度）
```

---

## 结果信息

执行完成后返回：

```cpp
// Result 内容
total_elapsed_time  // 总耗时
success_phase       // 完成的阶段（1/2/3）
total_distance      // 总行驶距离（米）
error_code          // 错误码（0=成功，1=碰撞，2=超时）
```

---

## 安全特性

1. **碰撞检测**：执行过程中持续进行前瞻碰撞检测
2. **超时保护**：超过 `time_allowance` 时间后自动终止
3. **参数验证**：启动时验证所有参数的有效性
4. **平滑控制**：速度平滑变化，避免突变

---

## 相关文件

| 文件 | 路径 |
|------|------|
| 参数配置指南 | `THREE_POINT_TURN_PARAMS.md` |
| API文档 | `docs/ThreePointTurn_API.md` |
| 故障排查指南 | `THREE_POINT_TURN_TROUBLESHOOTING.md` |
| 参数示例 | `examples/three_point_turn_params_example.yaml` |
| 启动脚本 | `examples/three_point_turn_test_launch.py` |
