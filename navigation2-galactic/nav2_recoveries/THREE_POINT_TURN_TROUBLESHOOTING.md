# ThreePointTurn 故障排查指南

## 概述

本指南提供ThreePointTurn三点掉头恢复行为的常见问题诊断和解决方案。

---

## 诊断工具

### 查看日志

```bash
# 查看behavior_server日志
ros2 node info /behavior_server

# 查看实时日志
ros2 run rqt_console rqt_console

# 命令行查看日志
ros2 topic echo /rosout | grep ThreePointTurn
```

### 查看话题

```bash
# 查看阿克曼指令
ros2 topic echo /ackermann_cmd

# 查看Action反馈
ros2 topic echo /ThreePointTurn/_action/feedback

# 查看代价地图
ros2 topic echo /local_costmap/costmap_raw
```

### 查看TF树

```bash
# 生成TF树可视化
ros2 run tf2_tools view_frames

# 查看特定TF
ros2 run tf2_ros tf2_echo base_footprint odom_combined
```

---

## 常见错误及解决方案

### 1. 插件加载失败

**错误信息**:
```
Failed to load plugin: nav2_behaviors/ThreePointTurn
```

**可能原因**:
- 插件未正确注册
- 库文件未编译或未找到
- 行为树插件库未在plugin_lib_names中配置

**解决方案**:

1. 检查导航参数文件中的plugin_lib_names配置：
```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
    # ... 其他插件 ...
    - nav2_three_point_turn_action_bt_node  # 确保此行存在
```

2. 检查behavior_plugins配置：
```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "three_point_turn"]
    three_point_turn:
      plugin: "nav2_behaviors/ThreePointTurn"
```

3. 重新编译：
```bash
cd /home/w20/default_WheelTec_ros2
colcon build --packages-select nav2_recoveries nav2_behavior_tree nav2_msgs
source install/setup.bash
```

---

### 2. Action服务器未启动

**错误信息**:
```
Action client failed to send goal: action server not available
```

**可能原因**:
- behavior_server未启动
- ThreePointTurn插件未正确加载
- Action服务器名称不匹配

**解决方案**:

1. 检查behavior_server是否运行：
```bash
ros2 node list | grep behavior_server
```

2. 检查Action服务器列表：
```bash
ros2 action list
```

应该看到 `/ThreePointTurn` 在列表中。

3. 检查Action服务器信息：
```bash
ros2 action info /ThreePointTurn
```

---

### 3. TF变换失败

**错误信息**:
```
Initial robot pose is not available.
Current robot pose is not available.
```

**可能原因**:
- TF树不完整
- global_frame或robot_base_frame配置错误
- transform_tolerance过小

**解决方案**:

1. 检查TF树：
```bash
ros2 run tf2_tools view_frames
# 打开生成的frames.pdf检查TF树结构
```

2. 检查TF变换：
```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

3. 检查导航参数中的frame配置：
```yaml
behavior_server:
  ros__parameters:
    global_frame: odom_combined
    robot_base_frame: base_footprint
    transform_tolerance: 0.1
```

---

### 4. 参数读取失败

**错误信息**:
```
Failed to get parameter: wheelbase
```

**可能原因**:
- 参数文件未正确加载
- 参数命名空间错误
- 参数未声明

**解决方案**:

1. 检查参数文件是否存在：
```bash
ls /home/w20/default_WheelTec_ros2/src/wheeltec_robot_nav2/param/three_point_turn_recoveries.yaml
```

2. 检查参数是否被加载：
```bash
ros2 param list | grep three_point_turn
```

3. 检查参数值：
```bash
ros2 param get /behavior_server three_point_turn_recoveries.wheelbase
```

4. 确保参数文件在launch文件中被正确加载。

---

### 5. 碰撞检测失败

**错误信息**:
```
Collision detected in phase X
ThreePointTurn failed: error_code=1
```

**可能原因**:
- 代价地图中有障碍物
- 机器人足迹配置不正确
- 前瞻模拟时间过长
- 空间确实不足

**解决方案**:

1. 查看代价地图：
```bash
ros2 run nav2_costmap_2d costmap_visualizer
```

2. 检查机器人足迹配置：
```yaml
local_costmap:
  ros__parameters:
    footprint: "[ [-0.09, -0.185], [-0.09, 0.185], [0.4,0.185], [0.4, -0.185] ]"
```

3. 减小前瞻模拟时间：
```yaml
three_point_turn_recoveries:
  ros__parameters:
    simulate_ahead_time: 1.0  # 从2.0减小到1.0
```

4. 清除代价地图：
```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

---

### 6. 执行超时

**错误信息**:
```
ThreePointTurn timeout: error_code=2
```

**可能原因**:
- time_allowance设置过小
- 速度设置过慢
- 前进/后退距离设置过大

**解决方案**:

1. 增加超时时间：
```xml
<!-- 在行为树中设置 -->
<ThreePointTurn forward_dist="0.75" backward_dist="1.0" ... time_allowance="15"/>
```

或在Action中设置：
```python
goal.time_allowance.sec = 15  # 增加到15秒
```

2. 增加速度：
```yaml
three_point_turn_recoveries:
  ros__parameters:
    min_linear_vel: 0.15  # 增加最小速度
```

或在行为树中设置：
```xml
<ThreePointTurn ... max_speed="0.8"/>
```

3. 减小前进/后退距离：
```xml
<ThreePointTurn forward_dist="0.5" backward_dist="0.7" .../>
```

---

### 7. 掉头不完整

**现象**: 三点掉头后朝向变化未达到180°

**可能原因**:
- 前进/后退距离不足
- 最大转向角过小
- 车辆参数（轴距）配置不正确

**解决方案**:

1. 增加前进和后退距离：
```xml
<ThreePointTurn forward_dist="1.0" backward_dist="1.5" .../>
```

2. 增加最大转向角：
```xml
<ThreePointTurn ... max_steering_angle="0.6"/>
```

3. 检查轴距配置：
```bash
ros2 param get /behavior_server three_point_turn_recoveries.wheelbase
```

4. 确保有足够的执行空间。

---

### 8. 机器人不移动

**现象**: ThreePointTurn执行但机器人不移动

**可能原因**:
- /ackermann_cmd话题未正确订阅
- 底层驱动未启动
- 速度值过小

**解决方案**:

1. 检查/ackermann_cmd话题：
```bash
ros2 topic info /ackermann_cmd
ros2 topic echo /ackermann_cmd
```

2. 检查话题订阅者：
```bash
ros2 node list
# 确认底盘控制节点正在运行
```

3. 启动底盘：
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

4. 检查速度值设置：
```bash
ros2 param get /behavior_server three_point_turn_recoveries.min_linear_vel
```

---

### 9. 参数无效错误

**错误信息**:
```
Invalid distance parameters
Invalid max_speed: X.XX
Requested steering angle exceeds maximum
```

**可能原因**:
- 参数值超出有效范围
- Goal参数与配置参数冲突

**解决方案**:

1. 检查参数有效范围：
   - `forward_dist` > 0
   - `backward_dist` > 0
   - `max_speed` ∈ (0, 2.0]
   - `max_steering_angle` ∈ [-max_steering_angle, max_steering_angle]

2. 调整行为树参数：
```xml
<ThreePointTurn forward_dist="0.75" backward_dist="1.0"
               max_speed="0.5" max_steering_angle="0.5"/>
```

3. 确保max_steering_angle不超过配置值：
```bash
ros2 param get /behavior_server three_point_turn_recoveries.max_steering_angle
```

---

## 调试技巧

### 1. 启用详细日志

```bash
# 设置RCUTILS_LOGGING_SEVERITY
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 或在launch文件中设置
<env name="RCUTILS_LOGGING_SEVERITY" value="DEBUG"/>
```

### 2. 使用rqt_console

```bash
ros2 run rqt_console rqt_console
```

可以过滤查看ThreePointTurn相关日志。

### 3. 录制和分析bag

```bash
# 录制相关话题
ros2 bag record -o three_point_turn_debug \
  /ackermann_cmd \
  /ThreePointTurn/_action/feedback \
  /local_costmap/costmap_raw \
  /rosout

# 回放
ros2 bag play three_point_turn_debug

# 分析
rqt_bag three_point_turn_debug
```

### 4. 单独测试Action

```python
# 编写测试脚本单独测试ThreePointTurn Action
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ThreePointTurn

def test_three_point_turn():
    rclpy.init()
    node = rclpy.create_node('test_three_point_turn')
    client = ActionClient(node, ThreePointTurn, 'ThreePointTurn')

    goal = ThreePointTurn.Goal()
    goal.forward_dist = 0.75
    goal.backward_dist = 1.0
    goal.max_speed = 0.5
    goal.max_steering_angle = 0.5
    goal.time_allowance.sec = 10

    client.wait_for_server()
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    # ... 处理结果
```

### 5. 检查阶段切换

查看日志中的阶段切换信息：
```
[INFO] [ThreePointTurn]: Transitioning to Phase 2
[INFO] [ThreePointTurn]: Transitioning to Phase 3
[INFO] [ThreePointTurn]: ThreePointTurn completed successfully
```

---

## 性能问题

### 执行时间过长

**可能原因**:
- 速度设置过慢
- 加速度限制过小
- 距离设置过大

**解决方案**:

1. 增加速度：
```yaml
three_point_turn_recoveries:
  ros__parameters:
    min_linear_vel: 0.15
    linear_acc_lim: 0.8
```

2. 减小距离：
```xml
<ThreePointTurn forward_dist="0.5" backward_dist="0.7" .../>
```

### CPU占用过高

**可能原因**:
- 碰撞检测频率过高
- 前瞻模拟时间过长

**解决方案**:

1. 减小前瞻时间：
```yaml
three_point_turn_recoveries:
  ros__parameters:
    simulate_ahead_time: 1.0
```

2. 降低behavior_server频率：
```yaml
behavior_server:
  ros__parameters:
    cycle_frequency: 10.0  # 从20.0降低到10.0
```

---

## 获取帮助

如果问题仍未解决：

1. 检查相关文档：
   - `README_ThreePointTurn.md` - 使用说明
   - `THREE_POINT_TURN_PARAMS.md` - 参数配置指南
   - `ThreePointTurn_API.md` - API文档

2. 查看Nav2官方文档：https://docs.nav2.org/

3. 检查相关源代码：
   - `nav2_recoveries/plugins/three_point_turn.cpp`
   - `nav2_behavior_tree/plugins/action/three_point_turn_action.cpp`

4. 查看单元测试示例：
   - `nav2_recoveries/test/test_three_point_turn.cpp`
