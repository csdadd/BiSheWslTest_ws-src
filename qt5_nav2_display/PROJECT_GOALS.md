# Nav2ViewWidget 类开发目标

## 1. 基本信息

| 项目 | 内容 |
|------|------|
| 类名 | `Nav2ViewWidget` |
| 功能包 | `qt5_nav2_display` |
| 框架 | Qt5 Widgets + ROS2 Galactic |
| 语言 | C++ |

## 2. 核心功能

### 2.1 地图显示
- 从本地加载 `map_server` 标准格式的地图文件（`.yaml` + `.png/.pgm`）
- 解析 `yaml` 中的 `image`、`origin`、`resolution` 参数
- **加载方式**：通过构造函数参数传入地图文件路径
- **显示方式**：地图在 Widget 中居中显示
- **尺寸控制**：Widget 根据地图尺寸自动调整大小，确保完整显示地图
- 不提供缩放/平移功能

### 2.2 路径显示
- 订阅 ROS2 话题：`/plan`（`nav_msgs/msg/Path`）
- 使用蓝色细线绘制规划路径
- 坐标转换：地图坐标系 → Qt 绘图坐标系

### 2.3 机器人显示
- 订阅 ROS2 话题：`/amcl_pose`（`geometry_msgs/msg/PoseWithCovarianceStamped`）
- **坐标系**：map 坐标系（AMCL 定位输出）
- **机器人形状**：矩形 + 矩形一边上延伸出的三角形表示车头方向
- **尺寸**：根据地图分辨率和实际机器人尺寸（长×宽）计算像素大小，尺寸参数可配置
- **初始状态**：未收到 `/amcl_pose` 数据时不显示机器人
- 实时更新机器人位置和朝向

### 2.4 导航目标显示
- 订阅 ROS2 话题：`/goal_pose`（`geometry_msgs/msg/PoseStamped`）
- 使用蓝色箭头表示目标位姿
- **箭头样式**：固定像素大小（如 20px），同时显示位置和朝向

### 2.5 交互功能
- **设置目标位置**：鼠标左键按下的位置确定目标点
- **设置目标朝向**：从按下点向拖动点的方向为目标朝向
- 松开鼠标时发布目标位姿到 ROS2 话题：`/goal_pose`

## 3. 设计约束

| 约束 | 要求 |
|------|------|
| ROS2 节点 | 外部传入 `rclcpp::Node::SharedPtr`，类内部不创建节点 |
| 刷新机制 | 数据驱动刷新（ROS2 回调发射 Qt 信号，信号连接到 `update()` 槽） |
| 可移植性 | 类独立，可直接复制到其他 Qt + ROS2 项目使用 |
| 最小化修改 | 仅实现核心功能，不添加额外特性 |
| Widget 尺寸 | 构造函数中调用 `setFixedSize(map_image_.width(), map_image_.height())` |

## 4. 依赖

```
Qt5Widgets
Qt5Gui
nav_msgs
geometry_msgs
rclcpp
yaml-cpp
```

## 5. 不包含的功能

- 地图缩放/平移
- 多层地图显示
- 代价地图显示
- 粒子云显示
- 其他 RViz 类似功能
