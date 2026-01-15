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

#include <cmath>
#include <algorithm>
#include <memory>

#include "three_point_turn.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_recoveries
{

ThreePointTurn::ThreePointTurn()
: Recovery<ThreePointTurnAction>(),
  feedback_(std::make_shared<ThreePointTurnAction::Feedback>())
{
}

ThreePointTurn::~ThreePointTurn()
{
}

void ThreePointTurn::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 阿克曼参数
  nav2_util::declare_parameter_if_not_declared(
    node, "wheelbase", rclcpp::ParameterValue(0.5));
  node->get_parameter("wheelbase", wheelbase_);

  nav2_util::declare_parameter_if_not_declared(
    node, "max_steering_angle", rclcpp::ParameterValue(0.785));
  node->get_parameter("max_steering_angle", max_steering_angle_);

  min_turning_radius_ = wheelbase_ / tan(max_steering_angle_);

  // 运动参数
  nav2_util::declare_parameter_if_not_declared(
    node, "min_linear_vel", rclcpp::ParameterValue(0.1));
  node->get_parameter("min_linear_vel", min_linear_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node, "linear_acc_lim", rclcpp::ParameterValue(0.5));
  node->get_parameter("linear_acc_lim", linear_acc_lim_);

  nav2_util::declare_parameter_if_not_declared(
    node, "steering_angle_velocity", rclcpp::ParameterValue(1.0));
  node->get_parameter("steering_angle_velocity", steering_angle_velocity_);

  nav2_util::declare_parameter_if_not_declared(
    node, "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  // 创建AckermannDrive发布器
  ackermann_pub_ = node->template create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "ackermann_cmd", 10);
}

Status ThreePointTurn::onRun(const std::shared_ptr<const ThreePointTurnAction::Goal> command)
{
  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  command_forward_dist_ = command->forward_dist;
  command_backward_dist_ = command->backward_dist;
  command_max_speed_ = command->max_speed;
  command_max_steering_angle_ = command->max_steering_angle;

  // 参数验证
  if (command_forward_dist_ <= 0.0 || command_backward_dist_ <= 0.0) {
    RCLCPP_ERROR(logger_, "Invalid distance parameters");
    return Status::FAILED;
  }

  if (command_max_speed_ <= 0.0 || command_max_speed_ > 2.0) {
    RCLCPP_ERROR(logger_, "Invalid max_speed: %.2f", command_max_speed_);
    return Status::FAILED;
  }

  if (std::abs(command_max_steering_angle_) > max_steering_angle_) {
    RCLCPP_ERROR(logger_, "Requested steering angle exceeds maximum");
    return Status::FAILED;
  }

  // 初始化状态机
  current_phase_ = 1;
  total_distance_traveled_ = 0.0;
  current_steering_angle_ = 0.0;

  initial_yaw_ = tf2::getYaw(initial_pose_.pose.orientation);
  phase_start_yaw_ = initial_yaw_;
  phase_start_pose_ = initial_pose_;

  // 计算阶段转向角
  phase2_steering_angle_ = -command_max_steering_angle_ * 0.7;
  phase3_steering_angle_ = command_max_steering_angle_ * 0.7;

  RCLCPP_INFO(logger_, "Starting ThreePointTurn: forward=%.2fm, backward=%.2fm",
              command_forward_dist_, command_backward_dist_);

  return Status::SUCCEEDED;
}

Status ThreePointTurn::onCycleUpdate()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  const double distance_diff = calculateDistance(phase_start_pose_, current_pose);
  const double yaw_diff = normalizeAngle(current_yaw - phase_start_yaw_);

  // 更新feedback
  feedback_->current_phase = current_phase_;
  feedback_->distance_traveled = total_distance_traveled_ + distance_diff;
  feedback_->current_yaw = normalizeAngle(current_yaw - initial_yaw_);
  feedback_->current_steering_angle = current_steering_angle_;
  action_server_->publish_feedback(feedback_);

  // 状态机
  Status phase_status;
  switch (current_phase_) {
    case 1:
      phase_status = executePhase1(current_pose, current_yaw, distance_diff);
      if (phase_status == Status::SUCCEEDED) {
        transitionToPhase(2, current_pose, current_yaw);
        return Status::RUNNING;
      }
      return phase_status;

    case 2:
      phase_status = executePhase2(current_pose, current_yaw, yaw_diff);
      if (phase_status == Status::SUCCEEDED) {
        transitionToPhase(3, current_pose, current_yaw);
        return Status::RUNNING;
      }
      return phase_status;

    case 3:
      phase_status = executePhase3(current_pose, current_yaw, yaw_diff);
      if (phase_status == Status::SUCCEEDED) {
        stopRobot();
        RCLCPP_INFO(logger_, "ThreePointTurn completed successfully");
        return Status::SUCCEEDED;
      }
      return phase_status;

    default:
      return Status::FAILED;
  }
}

Status ThreePointTurn::executePhase1(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double current_yaw,
  double distance_diff)
{
  if (distance_diff >= command_forward_dist_) {
    return Status::SUCCEEDED;
  }

  double remaining_dist = command_forward_dist_ - distance_diff;
  double speed = calculateSmoothSpeed(remaining_dist, min_linear_vel_,
                                      command_max_speed_, linear_acc_lim_);

  auto cmd = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
  cmd->speed = speed;
  cmd->steering_angle = 0.0;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = current_yaw;

  if (!isCollisionFree(remaining_dist, cmd.get(), pose2d, true)) {
    stopRobot();
    return Status::FAILED;
  }

  ackermann_pub_->publish(std::move(cmd));
  return Status::RUNNING;
}

Status ThreePointTurn::executePhase2(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double current_yaw,
  double yaw_diff)
{
  const double target_yaw_change = M_PI / 2;

  if (std::abs(yaw_diff) >= target_yaw_change) {
    return Status::SUCCEEDED;
  }

  double speed = -calculateSmoothSpeed(std::abs(yaw_diff), min_linear_vel_,
                                       command_max_speed_, linear_acc_lim_);

  auto cmd = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
  cmd->speed = speed;
  cmd->steering_angle = phase2_steering_angle_;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = current_yaw;

  if (!isCollisionFree(target_yaw_change, cmd.get(), pose2d, false)) {
    stopRobot();
    return Status::FAILED;
  }

  ackermann_pub_->publish(std::move(cmd));
  current_steering_angle_ = phase2_steering_angle_;
  return Status::RUNNING;
}

Status ThreePointTurn::executePhase3(
  const geometry_msgs::msg::PoseStamped & current_pose,
  double current_yaw,
  double yaw_diff)
{
  const double target_yaw_change = M_PI / 2;

  if (std::abs(yaw_diff) >= target_yaw_change) {
    return Status::SUCCEEDED;
  }

  double remaining_yaw = target_yaw_change - std::abs(yaw_diff);
  double speed = calculateSmoothSpeed(remaining_yaw, min_linear_vel_,
                                      command_max_speed_, linear_acc_lim_);

  auto cmd = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
  cmd->speed = speed;
  cmd->steering_angle = phase3_steering_angle_;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = current_yaw;

  if (!isCollisionFree(target_yaw_change, cmd.get(), pose2d, true)) {
    stopRobot();
    return Status::FAILED;
  }

  ackermann_pub_->publish(std::move(cmd));
  current_steering_angle_ = phase3_steering_angle_;
  return Status::RUNNING;
}

double ThreePointTurn::calculateSmoothSpeed(
  double remaining, double min_vel, double max_vel, double acc_lim)
{
  double vel = sqrt(2 * acc_lim * remaining);
  return std::min(std::max(vel, min_vel), max_vel);
}

double ThreePointTurn::calculateDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2)
{
  double dx = pose2.pose.position.x - pose1.pose.position.x;
  double dy = pose2.pose.position.y - pose1.pose.position.y;
  return sqrt(dx * dx + dy * dy);
}

double ThreePointTurn::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

void ThreePointTurn::transitionToPhase(
  uint8_t new_phase,
  const geometry_msgs::msg::PoseStamped & current_pose,
  double current_yaw)
{
  double distance_diff = calculateDistance(phase_start_pose_, current_pose);
  total_distance_traveled_ += distance_diff;
  current_phase_ = new_phase;
  phase_start_pose_ = current_pose;
  phase_start_yaw_ = current_yaw;
  RCLCPP_INFO(logger_, "Transitioning to Phase %d", new_phase);
}

void ThreePointTurn::stopRobot()
{
  auto cmd = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
  cmd->speed = 0.0;
  cmd->steering_angle = 0.0;
  ackermann_pub_->publish(std::move(cmd));
  Recovery<ThreePointTurnAction>::stopRobot();
}

bool ThreePointTurn::isCollisionFree(
  const double & remaining,
  ackermann_msgs::msg::AckermannDrive * cmd,
  geometry_msgs::msg::Pose2D & pose2d,
  bool check_by_distance)
{
  int cycle_count = 0;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;

  while (cycle_count < max_cycle_count) {
    double dt = cycle_count / cycle_frequency_;

    double v = cmd->speed;
    double delta = cmd->steering_angle;
    double theta_dot = (v / wheelbase_) * tan(delta);

    pose2d.theta = init_pose.theta + theta_dot * dt;
    pose2d.x = init_pose.x + v * cos(pose2d.theta) * dt;
    pose2d.y = init_pose.y + v * sin(pose2d.theta) * dt;

    cycle_count++;

    if (check_by_distance) {
      double dx = pose2d.x - init_pose.x;
      double dy = pose2d.y - init_pose.y;
      double dist = sqrt(dx * dx + dy * dy);
      if (remaining - dist <= 0.0) break;
    } else {
      double yaw_change = normalizeAngle(pose2d.theta - init_pose.theta);
      if (std::abs(remaining) - std::abs(yaw_change) <= 0.0) break;
    }

    if (!collision_checker_->isCollisionFree(pose2d)) {
      return false;
    }
  }
  return true;
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::ThreePointTurn, nav2_core::Recovery)
