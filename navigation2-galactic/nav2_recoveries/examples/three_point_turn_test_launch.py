#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ThreePointTurn 三点掉头测试启动脚本

该脚本用于启动一个简化的导航系统，专门用于测试ThreePointTurn恢复行为。

使用方法：
    ros2 launch nav2_recoveries three_point_turn_test_launch.py

可选参数：
    forward_dist: 前进距离（米），默认0.75
    backward_dist: 后退距离（米），默认1.0
    max_speed: 最大速度（m/s），默认0.5
    max_steering_angle: 最大转向角（弧度），默认0.5
    use_sim_time: 是否使用仿真时间，默认true

示例：
    # 使用默认参数
    ros2 launch nav2_recoveries three_point_turn_test_launch.py

    # 自定义参数
    ros2 launch nav2_recoveries three_point_turn_test_launch.py forward_dist:=0.5 max_speed:=0.3
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    nav2_recoveries_dir = get_package_share_directory('nav2_recoveries')
    nav2_behavior_tree_dir = get_package_share_directory('nav2_behavior_tree')

    # 声明启动参数
    forward_dist_arg = DeclareLaunchArgument(
        'forward_dist',
        default_value='0.75',
        description='前进距离（米）'
    )

    backward_dist_arg = DeclareLaunchArgument(
        'backward_dist',
        default_value='1.0',
        description='后退距离（米）'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='最大速度（m/s）'
    )

    max_steering_angle_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.5',
        description='最大转向角（弧度）'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    # ThreePointTurn恢复行为节点
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'costmap_topic': 'local_costmap/costmap_raw',
                'footprint_topic': 'local_costmap/published_footprint',
                'cycle_frequency': 10.0,
                'behavior_plugins': ['three_point_turn'],
                'three_point_turn': {
                    'plugin': 'nav2_behaviors/ThreePointTurn',
                    'wheelbase': 0.5,
                    'max_steering_angle': 0.5,
                    'min_linear_vel': 0.1,
                    'linear_acc_lim': 0.5,
                    'steering_angle_velocity': 1.0,
                    'simulate_ahead_time': 2.0,
                },
                'global_frame': 'odom_combined',
                'robot_base_frame': 'base_footprint',
                'transform_tolerance': 0.1,
            }
        ]
    )

    # 测试客户端节点
    test_client_node = Node(
        package='nav2_recoveries',
        executable='three_point_turn_test_client',
        name='three_point_turn_test_client',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'forward_dist': LaunchConfiguration('forward_dist'),
                'backward_dist': LaunchConfiguration('backward_dist'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            }
        ]
    )

    # 延迟启动测试客户端，等待behavior_server就绪
    delayed_test_client = TimerAction(
        period=3.0,
        actions=[test_client_node]
    )

    return LaunchDescription([
        use_sim_time_arg,
        forward_dist_arg,
        backward_dist_arg,
        max_speed_arg,
        max_steering_angle_arg,
        behavior_server_node,
        delayed_test_client,
    ])


"""
===========================================
测试客户端实现（three_point_turn_test_client.cpp）
===========================================

如果需要编译测试客户端，可以在 nav2_recoveries/src/ 中添加以下代码：

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/three_point_turn.hpp>

using namespace std::chrono_literals;
using ThreePointTurn = nav2_msgs::action::ThreePointTurn;

class ThreePointTurnTestClient : public rclcpp::Node
{
public:
  ThreePointTurnTestClient()
  : Node("three_point_turn_test_client")
  {
    // 获取参数
    this->declare_parameter<double>("forward_dist", 0.75);
    this->declare_parameter<double>("backward_dist", 1.0);
    this->declare_parameter<double>("max_speed", 0.5);
    this->declare_parameter<double>("max_steering_angle", 0.5);

    auto forward_dist = this->get_parameter("forward_dist").as_double();
    auto backward_dist = this->get_parameter("backward_dist").as_double();
    auto max_speed = this->get_parameter("max_speed").as_double();
    auto max_steering_angle = this->get_parameter("max_steering_angle").as_double();

    // 创建Action客户端
    action_client_ = rclcpp_action::create_client<ThreePointTurn>(
      this,
      "ThreePointTurn"
    );

    // 等待服务器就绪
    RCLCPP_INFO(this->get_logger(), "Waiting for ThreePointTurn action server...");
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      return;
    }

    // 发送目标
    RCLCPP_INFO(this->get_logger(), "Sending ThreePointTurn goal...");
    auto goal_msg = ThreePointTurn::Goal();
    goal_msg.forward_dist = forward_dist;
    goal_msg.backward_dist = backward_dist;
    goal_msg.max_speed = max_speed;
    goal_msg.max_steering_angle = max_steering_angle;
    goal_msg.time_allowance.sec = 10;

    auto goal_future = action_client_->async_send_goal(goal_msg);

    // 等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal!");
      return;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
      return;
    }

    // 获取结果
    auto result_future = action_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result!");
      return;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "ThreePointTurn SUCCEEDED!");
      RCLCPP_INFO(this->get_logger(), "Total distance: %.2f m", result.result->total_distance);
      RCLCPP_INFO(this->get_logger(), "Total time: %d s", result.result->total_elapsed_time.sec);
    } else {
      RCLCPP_ERROR(this->get_logger(), "ThreePointTurn FAILED! Error code: %d", result.result->error_code);
    }

    rclcpp::shutdown();
  }

private:
  rclcpp_action::Client<ThreePointTurn>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThreePointTurnTestClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

并在 CMakeLists.txt 中添加：

add_executable(three_point_turn_test_client src/test/three_point_turn_test_client.cpp)
ament_target_dependencies(three_point_turn_test_client
  rclcpp
  rclcpp_action
  nav2_msgs
)

 install(TARGETS
   three_point_turn_test_client
   DESTINATION lib/${PROJECT_NAME}
 )
"""

"""
===========================================
简化测试方法（Python）
===========================================

如果不想编译测试客户端，可以使用以下Python脚本直接测试：

#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ThreePointTurn
from builtin_interfaces.msg import Duration

def main():
    rclpy.init()
    node = rclpy.create_node('three_point_turn_test_client')

    # 创建Action客户端
    action_client = ActionClient(node, ThreePointTurn, 'ThreePointTurn')

    # 等待服务器
    node.get_logger().info('Waiting for ThreePointTurn action server...')
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Action server not available!')
        return

    # 发送目标
    node.get_logger().info('Sending ThreePointTurn goal...')
    goal = ThreePointTurn.Goal()
    goal.forward_dist = 0.75
    goal.backward_dist = 1.0
    goal.max_speed = 0.5
    goal.max_steering_angle = 0.5
    goal.time_allowance = Duration(sec=10, nanosec=0)

    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle:
        node.get_logger().error('Goal was rejected!')
        return

    # 获取结果
    result_future = action_client.get_result_async(goal_handle)
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()
    if result.code == rclcpp_action.ResultCode.SUCCEEDED:
        node.get_logger().info('ThreePointTurn SUCCEEDED!')
        node.get_logger().info(f'Total distance: {result.result.total_distance} m')
        node.get_logger().info(f'Total time: {result.result.total_elapsed_time.sec} s')
    else:
        node.get_logger().error(f'ThreePointTurn FAILED! Error code: {result.result.error_code}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

"""
===========================================
集成到现有导航系统
===========================================

要将ThreePointTurn集成到现有导航系统：

1. 确保参数文件已配置：
   src/wheeltec_robot_nav2/param/three_point_turn_recoveries.yaml

2. 确保导航参数文件中包含ThreePointTurn：
   behavior_server:
     ros__parameters:
       behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "three_point_turn"]
       three_point_turn:
         plugin: "nav2_behaviors/ThreePointTurn"

3. 确保行为树包含ThreePointTurn节点：
   <ThreePointTurn forward_dist="0.75" backward_dist="1.0"
                  max_speed="0.5" max_steering_angle="0.5"/>

4. 启动导航系统：
   ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

ThreePointTurn将在导航失败时作为恢复行为自动触发。
"""
