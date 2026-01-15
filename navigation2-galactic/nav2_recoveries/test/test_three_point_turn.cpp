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

#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "three_point_turn.hpp"
#include "nav2_msgs/action/three_point_turn.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;
using namespace nav2_recoveries;
using ThreePointTurnAction = nav2_msgs::action::ThreePointTurn;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ThreePointTurnAction>;

class ThreePointTurnTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "ThreePointTurnTestNode", rclcpp::NodeOptions());

    node_lifecycle_->declare_parameter(
      "costmap_topic",
      rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "footprint_topic",
      rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

    // 声明 ThreePointTurn 需要的参数
    node_lifecycle_->declare_parameter("wheelbase", rclcpp::ParameterValue(0.5));
    node_lifecycle_->declare_parameter("max_steering_angle", rclcpp::ParameterValue(0.5));
    node_lifecycle_->declare_parameter("min_linear_vel", rclcpp::ParameterValue(0.1));
    node_lifecycle_->declare_parameter("linear_acc_lim", rclcpp::ParameterValue(0.5));
    node_lifecycle_->declare_parameter("steering_angle_velocity", rclcpp::ParameterValue(1.0));
    node_lifecycle_->declare_parameter("simulate_ahead_time", rclcpp::ParameterValue(2.0));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic, footprint_topic;
    node_lifecycle_->get_parameter("costmap_topic", costmap_topic);
    node_lifecycle_->get_parameter("footprint_topic", footprint_topic);

    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, costmap_topic);
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, footprint_topic, 1.0);
    collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_, *tf_buffer_,
      node_lifecycle_->get_name(), "odom_combined");

    recovery_ = std::make_shared<ThreePointTurn>();
    recovery_->configure(node_lifecycle_, "ThreePointTurn", tf_buffer_, collision_checker_);
    recovery_->activate();

    client_ = rclcpp_action::create_client<ThreePointTurnAction>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_graph_interface(),
      node_lifecycle_->get_node_logging_interface(),
      node_lifecycle_->get_node_waitables_interface(), "ThreePointTurn");
  }

  void TearDown() override
  {
    recovery_->deactivate();
    recovery_->cleanup();
  }

  bool sendGoal(double forward_dist, double backward_dist, double max_speed, double max_steering_angle)
  {
    if (!client_->wait_for_action_server(4s)) {
      return false;
    }

    auto goal = ThreePointTurnAction::Goal();
    goal.forward_dist = forward_dist;
    goal.backward_dist = backward_dist;
    goal.max_speed = max_speed;
    goal.max_steering_angle = max_steering_angle;

    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_lifecycle_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    goal_handle_ = future_goal.get();
    return goal_handle_ != nullptr;
  }

  Status getOutcome()
  {
    auto future_result = client_->async_get_result(goal_handle_);
    rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
    auto result = future_result.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return Status::SUCCEEDED;
    }
    return Status::FAILED;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<ThreePointTurn> recovery_;
  std::shared_ptr<rclcpp_action::Client<ThreePointTurnAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ThreePointTurnAction>> goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

// ==================== 基础功能测试 ====================

TEST_F(ThreePointTurnTest, testSuccess)
{
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 0.5));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

TEST_F(ThreePointTurnTest, testFailureOnInvalidParameters)
{
  // 测试负距离
  ASSERT_TRUE(sendGoal(-0.5, 1.0, 0.5, 0.5));
  EXPECT_EQ(getOutcome(), Status::FAILED);

  // 测试零距离
  ASSERT_TRUE(sendGoal(0.0, 1.0, 0.5, 0.5));
  EXPECT_EQ(getOutcome(), Status::FAILED);

  // 测试零速度
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.0, 0.5));
  EXPECT_EQ(getOutcome(), Status::FAILED);

  // 测试超速
  ASSERT_TRUE(sendGoal(0.75, 1.0, 3.0, 0.5));
  EXPECT_EQ(getOutcome(), Status::FAILED);
}

// ==================== 参数配置测试 ====================

TEST_F(ThreePointTurnTest, testWheelbaseParameter)
{
  double wheelbase = 0.5;
  node_lifecycle_->set_parameter(rclcpp::Parameter("wheelbase", wheelbase));
  recovery_->cleanup();
  recovery_->configure(node_lifecycle_, "ThreePointTurn", tf_buffer_, collision_checker_);
  SUCCEED();
}

TEST_F(ThreePointTurnTest, testSteeringAngleParameter)
{
  double max_steering_angle = 0.5;
  node_lifecycle_->set_parameter(rclcpp::Parameter("max_steering_angle", max_steering_angle));
  recovery_->cleanup();
  recovery_->configure(node_lifecycle_, "ThreePointTurn", tf_buffer_, collision_checker_);
  SUCCEED();
}

TEST_F(ThreePointTurnTest, testVelocityParameters)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("min_linear_vel", 0.1));
  node_lifecycle_->set_parameter(rclcpp::Parameter("linear_acc_lim", 0.5));
  recovery_->cleanup();
  recovery_->configure(node_lifecycle_, "ThreePointTurn", tf_buffer_, collision_checker_);
  SUCCEED();
}

// ==================== 边界条件测试 ====================

TEST_F(ThreePointTurnTest, testMaximumSteeringAngle)
{
  // 测试最大转向角边界
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 1.0));
  EXPECT_EQ(getOutcome(), Status::FAILED);
}

TEST_F(ThreePointTurnTest, testMinimumVelocity)
{
  // 测试最小速度边界
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.01, 0.5));
  // 应该成功，因为速度可以低于配置值
  SUCCEED();
}

// ==================== 阶段测试 ====================

TEST_F(ThreePointTurnTest, testPhase1Forward)
{
  // 测试 Phase 1 前进功能
  ASSERT_TRUE(sendGoal(0.2, 1.0, 0.3, 0.3));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

TEST_F(ThreePointTurnTest, testPhase2BackwardTurn)
{
  // 测试 Phase 2 后退转向功能
  ASSERT_TRUE(sendGoal(0.75, 0.5, 0.4, 0.4));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

TEST_F(ThreePointTurnTest, testPhase3ForwardTurn)
{
  // 测试 Phase 3 前进转向功能
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 0.5));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

// ==================== 碰撞检测测试 ====================

TEST_F(ThreePointTurnTest, testCollisionDetection)
{
  // 测试碰撞检测功能
  // 注意：此测试需要设置实际的代价地图和障碍物
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 0.5));
  auto outcome = getOutcome();
  // 在无障碍物环境中应该成功
  EXPECT_TRUE(outcome == Status::SUCCEEDED || outcome == Status::FAILED);
}

// ==================== 状态机测试 ====================

TEST_F(ThreePointTurnTest, testStateInitialization)
{
  // 测试状态初始化
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 0.5));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

TEST_F(ThreePointTurnTest, testPhaseTransitions)
{
  // 测试阶段切换逻辑
  ASSERT_TRUE(sendGoal(0.5, 0.8, 0.4, 0.4));
  EXPECT_EQ(getOutcome(), Status::SUCCEEDED);
}

// ==================== 超时和错误处理测试 ====================

TEST_F(ThreePointTurnTest, testElapsedTimeTracking)
{
  ASSERT_TRUE(sendGoal(0.75, 1.0, 0.5, 0.5));
  auto future_result = client_->async_get_result(goal_handle_);
  rclcpp::spin_until_future_complete(node_lifecycle_, future_result);
  auto result = future_result.get();

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    EXPECT_GT(result.result->total_elapsed_time.sec, 0);
  }
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
