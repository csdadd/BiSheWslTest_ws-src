# ThreePointTurn API 文档

## 概述

ThreePointTurn恢复行为提供ROS2 Action接口和行为树节点接口，用于在Nav2导航框架中实现阿克曼小车的三点掉头功能。

---

## Action 消息接口

### Action 定义

**文件位置**: `src/navigation2-galactic/nav2_msgs/action/ThreePointTurn.action`

### Goal（目标）

客户端发送给服务器的目标参数。

| 字段名 | 类型 | 说明 | 默认值 | 范围 |
|--------|------|------|--------|------|
| `forward_dist` | `float` | Phase 1前进距离（米） | - | > 0 |
| `backward_dist` | `float` | Phase 2后退距离（米） | - | > 0 |
| `max_speed` | `float` | 最大速度（m/s） | - | (0, 2.0] |
| `max_steering_angle` | `float` | 最大转向角（弧度） | - | [-max, max] |
| `time_allowance` | `builtin_interfaces/Duration` | 超时时间 | - | > 0 |

**C++ 类型**:
```cpp
nav2_msgs::action::ThreePointTurn::Goal
```

**Python 类型**:
```python
nav2_msgs.action.ThreePointTurn.Goal
```

### Result（结果）

服务器返回给客户端的执行结果。

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `total_elapsed_time` | `builtin_interfaces/Duration` | 总执行耗时 |
| `success_phase` | `uint8` | 成功完成的阶段（1/2/3） |
| `total_distance` | `float` | 总行驶距离（米） |
| `error_code` | `int8` | 错误码（0=成功，1=碰撞，2=超时） |

**错误码定义**:
| 错误码 | 含义 |
|--------|------|
| 0 | 执行成功 |
| 1 | 碰撞检测失败 |
| 2 | 执行超时 |
| -1 | 其他错误（TF失败、参数无效等） |

### Feedback（反馈）

执行过程中实时发布的反馈信息。

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `current_phase` | `uint8` | 当前阶段（1/2/3） |
| `distance_traveled` | `float` | 已行驶距离（米） |
| `current_yaw` | `float` | 当前朝向角变化（弧度） |
| `current_steering_angle` | `float` | 当前转向角（弧度） |

---

## C++ API

### 恢复行为服务器类

**头文件**: `nav2_recoveries/plugins/three_point_turn.hpp`

```cpp
namespace nav2_recoveries {

class ThreePointTurn : public Recovery<ThreePointTurnAction>
{
public:
  ThreePointTurn();
  ~ThreePointTurn();

  // 配置参数
  void onConfigure() override;

  // 执行初始化
  Status onRun(const std::shared_ptr<const ThreePointTurnAction::Goal> command) override;

  // 周期更新
  Status onCycleUpdate() override;

protected:
  // 碰撞检测
  bool isCollisionFree(
    const double & remaining,
    ackermann_msgs::msg::AckermannDrive * cmd,
    geometry_msgs::msg::Pose2D & pose2d,
    bool check_by_distance);

  // 速度平滑计算
  double calculateSmoothSpeed(double remaining, double min_vel, double max_vel, double acc_lim);

  // 距离计算
  double calculateDistance(const geometry_msgs::msg::PoseStamped & pose1,
                           const geometry_msgs::msg::PoseStamped & pose2);

  // 角度归一化
  double normalizeAngle(double angle);

  // 阶段切换
  void transitionToPhase(uint8_t new_phase,
                         const geometry_msgs::msg::PoseStamped & current_pose,
                         double current_yaw);

  // 停止机器人
  void stopRobot();

  // 各阶段执行
  Status executePhase1(const geometry_msgs::msg::PoseStamped & current_pose,
                       double current_yaw, double distance_diff);
  Status executePhase2(const geometry_msgs::msg::PoseStamped & current_pose,
                       double current_yaw, double yaw_diff);
  Status executePhase3(const geometry_msgs::msg::PoseStamped & current_pose,
                       double current_yaw, double yaw_diff);

private:
  // 阿克曼参数
  double wheelbase_;
  double max_steering_angle_;
  double min_turning_radius_;

  // 运动参数
  double min_linear_vel_;
  double max_linear_vel_;
  double linear_acc_lim_;
  double steering_angle_velocity_;
  double simulate_ahead_time_;

  // 状态变量
  uint8_t current_phase_;
  double phase_start_distance_;
  double phase_start_yaw_;
  double initial_yaw_;
  double total_distance_traveled_;
  double current_steering_angle_;

  // 发布器
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;
};

}  // namespace nav2_recoveries
```

### 行为树节点类

**头文件**: `nav2_behavior_tree/plugins/action/three_point_turn_action.hpp`

```cpp
namespace nav2_behavior_tree {

class ThreePointTurnAction : public BtActionNode<nav2_msgs::action::ThreePointTurn>
{
public:
  ThreePointTurnAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts();
};

}  // namespace nav2_behavior_tree
```

---

## Python API

### Action Client 示例

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ThreePointTurn
from builtin_interfaces.msg import Duration

class ThreePointTurnClient:
    def __init__(self):
        self.node = rclpy.create_node('three_point_turn_client')
        self.action_client = ActionClient(self.node, ThreePointTurn, 'ThreePointTurn')

    def send_goal(self, forward_dist=0.75, backward_dist=1.0,
                  max_speed=0.5, max_steering_angle=0.5, timeout_sec=10):
        """发送三点掉头目标"""
        goal = ThreePointTurn.Goal()
        goal.forward_dist = float(forward_dist)
        goal.backward_dist = float(backward_dist)
        goal.max_speed = float(max_speed)
        goal.max_steering_angle = float(max_steering_angle)
        goal.time_allowance = Duration(sec=timeout_sec)

        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal)

    def get_feedback(self):
        """获取反馈"""
        # 在goal回调中使用
        future.result()._get_feedback_callback()
```

### 使用示例

```python
# 发送目标
client = ThreePointTurnClient()
future = client.send_goal(
    forward_dist=0.75,
    backward_dist=1.0,
    max_speed=0.5,
    max_steering_angle=0.5,
    timeout_sec=10
)
rclpy.spin_until_future_complete(client.node, future)
goal_handle = future.result()

# 获取结果
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(client.node, result_future)
result = result_future.result().result

print(f"执行结果: error_code={result.error_code}")
print(f"总耗时: {result.total_elapsed_time.sec}秒")
print(f"总距离: {result.total_distance}米")
```

---

## 行为树节点接口

### 节点类型

`ThreePointTurn` 是一个行为树Action节点，继承自 `BtActionNode`。

### 节点名称

```
ThreePointTurn
```

### 输入端口

| 端口名 | 类型 | 默认值 | 必需 | 说明 |
|--------|------|--------|------|------|
| `forward_dist` | float | 0.75 | 否 | 前进距离（米） |
| `backward_dist` | float | 1.0 | 否 | 后退距离（米） |
| `max_speed` | float | 0.5 | 否 | 最大速度（m/s） |
| `max_steering_angle` | float | 0.5 | 否 | 最大转向角（弧度） |
| `server_timeout` | int | 20 | 否 | 服务器超时（毫秒） |

### 返回值

| 返回值 | 说明 |
|--------|------|
| `BT::NodeStatus::SUCCESS` | 三点掉头成功完成 |
| `BT::NodeStatus::FAILURE` | 执行失败（碰撞、超时等） |
| `BT::NodeStatus::RUNNING` | 正在执行中 |

### 行为树使用示例

```xml
<ReactiveFallback name="RecoveryFallback">
  <GoalUpdated/>
  <RoundRobin name="RecoveryActions">
    <Sequence name="ClearingActions">
      <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
    </Sequence>
    <ThreePointTurn forward_dist="0.75" backward_dist="1.0"
                   max_speed="0.5" max_steering_angle="0.5"/>
  </RoundRobin>
</ReactiveFallback>
```

---

## 话题接口

### 发布的话题

| 话题名 | 消息类型 | QoS | 描述 |
|--------|----------|-----|------|
| `/ackermann_cmd` | `ackermann_msgs::msg::AckermannDrive` | 10 | 阿克曼驱动指令 |

**消息格式**:
```cpp
// ackermann_msgs::msg::AckermannDrive
float64 speed                   // 线速度（m/s），正值为前进
float64 steering_angle          // 转向角（弧度），正值向左
float64 steering_angle_velocity // 转向角速度（rad/s）
float64 acceleration            // 加速度（m/s²）
float64 jerk                    // 加速度变化率（m/s³）
```

### 订阅的话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/local_costmap/costmap_raw` | `nav2_msgs::msg::Costmap` | 局部代价地图（碰撞检测） |
| `/local_costmap/published_footprint` | `geometry_msgs::msg::PolygonStamped` | 机器人足迹（碰撞检测） |

---

## ROS服务接口

ThreePointTurn不提供直接的ROS服务接口，通过Action和Behavior Tree接口使用。

---

## 参数接口

### ROS2 参数

ThreePointTurn通过ROS2参数系统配置参数。

**参数命名空间**: `three_point_turn_recoveries`

**参数列表**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `wheelbase` | double | 0.5 | 轴距（米） |
| `max_steering_angle` | double | 0.785 | 最大转向角（弧度） |
| `min_linear_vel` | double | 0.1 | 最小线速度（m/s） |
| `linear_acc_lim` | double | 0.5 | 线加速度限制（m/s²） |
| `steering_angle_velocity` | double | 1.0 | 转向角速度（rad/s） |
| `simulate_ahead_time` | double | 2.0 | 前瞻模拟时间（秒） |

### 动态参数重配置

ThreePointTurn支持运行时参数更新：

```bash
# 设置新的轴距值
ros2 param set /behavior_server three_point_turn_recoveries.wheelbase 0.6

# 设置新的最大转向角
ros2 param set /behavior_server three_point_turn_recoveries.max_steering_angle 0.55
```

---

## 插件接口

### Pluginlib 注册

**插件类型**: `nav2_core::Recovery`

**插件名称**: `nav2_recoveries/ThreePointTurn`

**类名**: `nav2_recoveries::ThreePointTurn`

**注册文件**: `nav2_recoveries/recovery_plugin.xml`

```xml
<library path="nav2_three_point_turn_recovery">
  <class name="nav2_recoveries/ThreePointTurn"
         type="nav2_recoveries::ThreePointTurn"
         base_class_type="nav2_core::Recovery">
    <description>Three-point turn recovery behavior for Ackermann steering vehicles</description>
  </class>
</library>
```

### 行为树插件注册

**插件类型**: `BT::ActionNodeBase`

**插件名称**: `ThreePointTurn`

**类名**: `nav2_behavior_tree::ThreePointTurnAction`

**注册文件**: `nav2_behavior_tree/nav2_tree_nodes.xml`

```xml
<Action ID="ThreePointTurn">
  <input_port name="forward_dist">前进距离（米）</input_port>
  <input_port name="backward_dist">后退距离（米）</input_port>
  <input_port name="max_speed">最大速度</input_port>
  <input_port name="max_steering_angle">最大转向角（弧度）</input_port>
</Action>
```

---

## 依赖项

### 消息依赖

- `nav2_msgs` - Action消息定义
- `ackermann_msgs` - 阿克曼驱动消息
- `geometry_msgs` - 几何消息类型
- `builtin_interfaces` - 基础类型（Duration等）

### 包依赖

- `nav2_core` - Nav2核心接口
- `nav2_behavior_tree` - 行为树框架
- `nav2_costmap_2d` - 代价地图
- `nav2_util` - Nav2工具函数
- `rclcpp_action` - ROS2 Action C++客户端
- `pluginlib` - 插件系统
- `tf2_ros` - TF变换

---

## 编译信息

### 库名称

```
nav2_three_point_turn_recovery
nav2_three_point_turn_action_bt_node
```

### 编译依赖

- `ament_cmake`
- `ament_cmake_gtest`
- `nav2_msgs`
- `ackermann_msgs`
