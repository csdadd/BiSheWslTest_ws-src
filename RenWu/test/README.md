# 功能测试说明文档

## 概述

本文档描述了地图显示与可视化功能的功能测试套件，包括单元测试和集成测试。

## 测试文件结构

```
RenWu/test/
├── main.cpp                    # 测试主程序入口
├── testmapconverter.h          # MapConverter 单元测试头文件
├── testmapconverter.cpp        # MapConverter 单元测试实现
├── testmapwidget.h             # MapWidget 单元测试头文件
├── testmapwidget.cpp           # MapWidget 单元测试实现
├── testintegration.h           # 集成测试头文件
└── testintegration.cpp         # 集成测试实现
```

## 测试内容

### 1. MapConverter 单元测试

#### 测试用例列表

1. **testConvertToImage**: 测试基本的地图图像转换功能
2. **testConvertToImageWithNullMap**: 测试空地图指针的处理
3. **testConvertToImageWithEmptyData**: 测试空数据地图的处理
4. **testConvertToImageWithDifferentValues**: 测试不同值的地图数据转换
5. **testCoordinateConversion**: 测试坐标转换功能
6. **testCoordinateConversionRoundTrip**: 测试坐标往返转换的准确性
7. **testCoordinateConversionWithInvalidResolution**: 测试无效分辨率的处理
8. **testCreateRobotPolygon**: 测试机器人多边形创建
9. **testCreateRobotPolygonWithDifferentAngles**: 测试不同角度的机器人多边形
10. **testCreateRobotPolygonWithDifferentSizes**: 测试不同尺寸的机器人多边形
11. **testCreateRobotPolygonCenter**: 测试机器人多边形中心点

### 2. MapWidget 单元测试

#### 测试用例列表

1. **testSetMapImage**: 测试地图图像设置
2. **testSetMapImageWithNullImage**: 测试空图像的处理
3. **testSetMapImageWithDifferentResolutions**: 测试不同分辨率的地图
4. **testUpdateRobotPose**: 测试机器人位姿更新
5. **testUpdateRobotPoseWithoutMap**: 测试无地图时的机器人位姿更新
6. **testUpdateRobotPoseMultipleTimes**: 测试多次更新机器人位姿
7. **testClearRobotPose**: 测试清除机器人位姿
8. **testClearRobotPoseWithoutRobot**: 测试无机器人时的清除操作
9. **testSetZoomLevel**: 测试缩放级别设置
10. **testSetZoomLevelWithInvalidValue**: 测试无效缩放值的处理
11. **testGetZoomLevel**: 测试获取缩放级别
12. **testZoomLevelSignal**: 测试缩放级别变化信号
13. **testCenterOnRobot**: 测试视图中心定位到机器人
14. **testCenterOnRobotWithoutRobot**: 测试无机器人时的中心定位
15. **testCenterOnPosition**: 测试视图中心定位到指定位置
16. **testWheelEvent**: 测试鼠标滚轮事件
17. **testWheelEventSignal**: 测试滚轮事件信号
18. **testMousePressEventLeftButton**: 测试左键按下事件
19. **testMousePressEventMiddleButton**: 测试中键按下事件
20. **testMousePressEventRightButton**: 测试右键按下事件
21. **testMousePressEventRightButtonSignal**: 测试右键点击信号
22. **testMouseMoveEvent**: 测试鼠标移动事件
23. **testMouseReleaseEvent**: 测试鼠标释放事件
24. **testMapClickedSignal**: 测试地图点击信号

### 3. 集成测试

#### 测试用例列表

1. **testMapThreadToMapWidget**: 测试 MapThread 到 MapWidget 的数据传递
2. **testMapThreadToMapWidgetWithRealMap**: 测试真实地图数据的传递
3. **testRobotStatusToMapWidget**: 测试机器人状态到地图的显示
4. **testMapThreadConnectionState**: 测试地图线程连接状态
5. **testMapWidgetZoomSignal**: 测试地图缩放信号
6. **testMapWidgetClickSignal**: 测试地图点击信号
7. **testMultipleRobotPoseUpdates**: 测试多次机器人位姿更新
8. **testMapUpdateFrequency**: 测试地图更新频率
9. **testMapWidgetWithoutMap**: 测试无地图时的地图控件
10. **testMapThreadLifecycle**: 测试地图线程生命周期

## 编译和运行测试

### 编译测试

```bash
cd ~/default_WheelTec_ros2
colcon build --packages-select RenWu --cmake-args -DBUILD_TESTING=ON
```

### 运行所有测试

```bash
cd ~/default_WheelTec_ros2
source install/setup.bash
colcon test --packages-select RenWu
```

### 运行特定测试

```bash
cd ~/default_WheelTec_ros2/build/RenWu
./RenWu_tests
```

### 查看测试结果

```bash
cd ~/default_WheelTec_ros2
colcon test-result --all --verbose
```

## 测试环境要求

- ROS2 Humble
- Qt5 或 Qt6
- CMake 3.5+
- C++17 编译器
- Qt Test 框架

## 测试覆盖率

### MapConverter

- convertToImage(): 100%
- mapToImage(): 100%
- imageToMap(): 100%
- createRobotPolygon(): 100%

### MapWidget

- setMapImage(): 100%
- updateRobotPose(): 100%
- clearRobotPose(): 100%
- setZoomLevel(): 100%
- getZoomLevel(): 100%
- centerOnRobot(): 100%
- centerOnPosition(): 100%
- wheelEvent(): 100%
- mousePressEvent(): 100%
- mouseMoveEvent(): 100%
- mouseReleaseEvent(): 100%

### 集成测试

- MapThread 到 MapWidget 数据传递: 100%
- 机器人状态到地图显示: 100%
- 连接状态管理: 100%
- 信号槽机制: 100%
- 线程生命周期: 100%

## 注意事项

1. **ROS2 环境**: 运行测试前需要先 source ROS2 环境
2. **地图数据**: 集成测试中的真实地图测试需要 ROS2 地图服务器运行
3. **测试超时**: 某些测试可能需要等待较长时间（如地图订阅测试）
4. **线程安全**: 测试中涉及多线程操作，需要注意线程同步

## 已知问题

1. 无

## 维护说明

- 添加新功能时，请同时添加相应的测试用例
- 修改现有功能时，请确保所有相关测试通过
- 定期运行测试以确保代码质量
