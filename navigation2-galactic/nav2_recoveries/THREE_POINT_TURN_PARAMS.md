# ThreePointTurn 参数配置指南

## 配置文件位置

ThreePointTurn的参数配置文件位于：
```
src/wheeltec_robot_nav2/param/three_point_turn_recoveries.yaml
```

---

## 参数分类

### 1. 阿克曼小车参数

这些参数描述了车辆的物理特性，需要根据实际车型调整。

#### wheelbase（轴距）

| 参数 | 说明 |
|------|------|
| 参数名 | `wheelbase` |
| 单位 | 米（m） |
| 默认值 | 0.5 |
| 典型范围 | 0.3 - 1.0 |

**说明**：前后轮之间的距离，直接影响转弯半径。

**推荐值**：
- mini系列：0.3 - 0.5
- senior系列：0.5 - 0.7
- top系列：0.6 - 1.0

#### max_steering_angle（最大转向角）

| 参数 | 说明 |
|------|------|
| 参数名 | `max_steering_angle` |
| 单位 | 弧度（rad） |
| 默认值 | 0.5 |
| 典型范围 | 0.4 - 0.8 |
| 角度值 | 约23° - 46° |

**说明**：前轮最大转向角度，正值向左，负值向右。

**换算公式**：
- 0.4 rad ≈ 23°
- 0.5 rad ≈ 29°
- 0.6 rad ≈ 34°
- 0.7 rad ≈ 40°
- 0.8 rad ≈ 46°

**推荐值**：
- 保守配置：0.4 - 0.5（转向平缓，空间需求大）
- 标准配置：0.5 - 0.6（平衡性能）
- 激进配置：0.6 - 0.8（转向急促，空间需求小）

#### min_turning_radius（最小转弯半径）

| 参数 | 说明 |
|------|------|
| 参数名 | `min_turning_radius` |
| 单位 | 米（m） |
| 计算方式 | `wheelbase / tan(max_steering_angle)` |
| 自动计算 | 是 |

**说明**：此参数会根据轴距和最大转向角自动计算，无需手动配置。

---

### 2. 运动参数

这些参数控制机器人的运动特性。

#### min_linear_vel（最小线速度）

| 参数 | 说明 |
|------|------|
| 参数名 | `min_linear_vel` |
| 单位 | 米/秒（m/s） |
| 默认值 | 0.1 |
| 典型范围 | 0.05 - 0.2 |

**说明**：机器人运动的最小速度，用于接近目标时减速。

**调优建议**：
- 精确要求高：0.05 - 0.1（速度慢，精度高）
- 标准配置：0.1 - 0.15
- 效率优先：0.15 - 0.2（速度快，精度略低）

#### linear_acc_lim（线加速度限制）

| 参数 | 说明 |
|------|------|
| 参数名 | `linear_acc_lim` |
| 单位 | 米/秒²（m/s²） |
| 默认值 | 0.5 |
| 典型范围 | 0.2 - 1.0 |

**说明**：控制速度变化的平滑程度，影响启停加减速。

**调优建议**：
- 平滑优先：0.2 - 0.4（加减速平缓，执行时间长）
- 标准配置：0.4 - 0.6
- 快速响应：0.6 - 1.0（响应快，但可能颠簸）

**速度计算公式**：
```
v = sqrt(2 * acc * remaining_distance)
```

#### steering_angle_velocity（转向角速度）

| 参数 | 说明 |
|------|------|
| 参数名 | `steering_angle_velocity` |
| 单位 | 弧度/秒（rad/s） |
| 默认值 | 1.0 |
| 典型范围 | 0.5 - 2.0 |

**说明**：转向轮转动的最大速度，影响转向响应速度。

**调优建议**：
- 慢速转向：0.5 - 1.0（平滑，但响应慢）
- 标准配置：1.0 - 1.5
- 快速转向：1.5 - 2.0（响应快，但可能颠簸）

---

### 3. 安全参数

#### simulate_ahead_time（前瞻模拟时间）

| 参数 | 说明 |
|------|------|
| 参数名 | `simulate_ahead_time` |
| 单位 | 秒（s） |
| 默认值 | 2.0 |
| 典型范围 | 1.0 - 3.0 |

**说明**：碰撞检测时向前模拟的时间长度。

**调优建议**：
- 狭窄空间：1.0 - 1.5（检测距离短，更灵活）
- 标准配置：1.5 - 2.0
- 开阔空间：2.0 - 3.0（检测距离长，更安全）

---

## 完整配置示例

```yaml
three_point_turn_recoveries:
  ros__parameters:
    # 阿克曼小车参数
    wheelbase: 0.5                # 轴距（米）
    max_steering_angle: 0.5       # 最大转向角（弧度，约29度）

    # 运动参数
    min_linear_vel: 0.1           # 最小线速度（m/s）
    linear_acc_lim: 0.5           # 线加速度限制（m/s²）
    steering_angle_velocity: 1.0  # 转向角速度（rad/s）

    # 安全参数
    simulate_ahead_time: 2.0      # 前瞻模拟时间（秒）
```

---

## 不同车型推荐配置

### Mini系列（小型机器人）

```yaml
three_point_turn_recoveries:
  ros__parameters:
    wheelbase: 0.35
    max_steering_angle: 0.55
    min_linear_vel: 0.08
    linear_acc_lim: 0.4
    steering_angle_velocity: 1.2
    simulate_ahead_time: 1.5
```

### Senior系列（中型机器人）

```yaml
three_point_turn_recoveries:
  ros__parameters:
    wheelbase: 0.5
    max_steering_angle: 0.5
    min_linear_vel: 0.1
    linear_acc_lim: 0.5
    steering_angle_velocity: 1.0
    simulate_ahead_time: 2.0
```

### Top系列（大型机器人）

```yaml
three_point_turn_recoveries:
  ros__parameters:
    wheelbase: 0.7
    max_steering_angle: 0.45
    min_linear_vel: 0.12
    linear_acc_lim: 0.6
    steering_angle_velocity: 0.8
    simulate_ahead_time: 2.5
```

---

## 行为树端口参数

在行为树XML中可以直接配置以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `forward_dist` | 0.75 | Phase 1前进距离（米），建议约1.5倍轴距 |
| `backward_dist` | 1.0 | Phase 2后退距离（米），建议约2倍轴距 |
| `max_speed` | 0.5 | 最大速度（m/s），不超过2.0 |
| `max_steering_angle` | 0.5 | 最大转向角（弧度），不超过配置值 |

**示例**：
```xml
<ThreePointTurn forward_dist="0.5" backward_dist="0.7"
               max_speed="0.3" max_steering_angle="0.45"/>
```

---

## 参数调优流程

### 1. 基础配置

1. 设置正确的轴距（从车辆规格获取）
2. 设置最大转向角（从车辆规格或实测获取）

### 2. 运动参数调整

1. 从推荐值开始
2. 在安全环境中测试
3. 根据实际表现调整：
   - 掉头不完整：增加前进/后退距离
   - 动作太慢：增加速度和加速度
   - 动作太颠簸：减小加速度和转向角速度

### 3. 安全参数调整

1. 根据环境调整前瞻时间
2. 狭窄环境减小，开阔环境增大
3. 确保碰撞检测正常工作

---

## 常见问题

### Q: 掉头不完整怎么办？

**A**: 可能原因：
1. 前进/后退距离不足 → 增加 `forward_dist` 和 `backward_dist`
2. 转向角过小 → 增加 `max_steering_angle`
3. 空间确实不足 → 考虑使用其他恢复行为

### Q: 动作太慢怎么办？

**A**: 调整以下参数：
1. 增加 `max_speed`
2. 增加 `linear_acc_lim`
3. 增加 `steering_angle_velocity`

### Q: 动作不平滑怎么办？

**A**: 调整以下参数：
1. 减小 `linear_acc_lim`
2. 减小 `steering_angle_velocity`
3. 增加 `min_linear_vel`

### Q: 经常碰撞检测失败怎么办？

**A**: 调整以下参数：
1. 减小 `simulate_ahead_time`
2. 检查代价地图配置
3. 确认机器人足迹参数正确
