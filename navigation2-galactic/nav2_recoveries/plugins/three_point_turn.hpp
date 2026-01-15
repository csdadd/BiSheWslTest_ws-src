// Copyright (c) 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_RECOVERIES__PLUGINS__THREE_POINT_TURN_HPP_
#define NAV2_RECOVERIES__PLUGINS__THREE_POINT_TURN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "nav2_msgs/action/three_point_turn.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_recoveries
{
using ThreePointTurnAction = nav2_msgs::action::ThreePointTurn;

class ThreePointTurn : public Recovery<ThreePointTurnAction>
{
public:
  ThreePointTurn();
  ~ThreePointTurn();

  Status onRun(const std::shared_ptr<const ThreePointTurnAction::Goal> command) override;
  void onConfigure() override;
  Status onCycleUpdate() override;

protected:
  bool isCollisionFree(
    const double & remaining,
    ackermann_msgs::msg::AckermannDrive * cmd,
    geometry_msgs::msg::Pose2D & pose2d,
    bool check_by_distance);

  double calculateSmoothSpeed(double remaining, double min_vel, double max_vel, double acc_lim);
  double calculateDistance(const geometry_msgs::msg::PoseStamped & pose1,
                           const geometry_msgs::msg::PoseStamped & pose2);
  double normalizeAngle(double angle);
  void transitionToPhase(uint8_t new_phase,
                         const geometry_msgs::msg::PoseStamped & current_pose,
                         double current_yaw);
  void stopRobot();

  Status executePhase1(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double current_yaw,
    double distance_diff);

  Status executePhase2(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double current_yaw,
    double yaw_diff);

  Status executePhase3(
    const geometry_msgs::msg::PoseStamped & current_pose,
    double current_yaw,
    double yaw_diff);

  ThreePointTurnAction::Feedback::SharedPtr feedback_;

  // 阿克曼参数
  double wheelbase_;
  double max_steering_angle_;
  double min_turning_radius_;

  // 运动参数
  double min_linear_vel_;
  double linear_acc_lim_;
  double steering_angle_velocity_;
  double simulate_ahead_time_;

  // Goal参数存储
  double command_forward_dist_;
  double command_backward_dist_;
  double command_max_speed_;
  double command_max_steering_angle_;

  // 状态变量
  uint8_t current_phase_;
  double phase_start_yaw_;
  double initial_yaw_;
  double total_distance_traveled_;
  double current_steering_angle_;

  // 位姿存储
  geometry_msgs::msg::PoseStamped initial_pose_;
  geometry_msgs::msg::PoseStamped phase_start_pose_;

  // AckermannDrive发布器
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;

  // 阶段特定的转向角
  double phase2_steering_angle_;
  double phase3_steering_angle_;
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__THREE_POINT_TURN_HPP_
