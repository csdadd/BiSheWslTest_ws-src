#!/bin/bash
# Nav2 View 启动脚本

# Source ROS2 环境
source /opt/ros/galactic/setup.bash
cd /home/w20/default_WheelTec_ros2
source install/setup.bash

# 运行程序
ros2 run qt5_nav2_display nav2_view "$@"
