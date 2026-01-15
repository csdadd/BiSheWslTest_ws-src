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

#include "nav2_behavior_tree/plugins/action/three_point_turn_action.hpp"

namespace nav2_behavior_tree
{

ThreePointTurnAction::ThreePointTurnAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ThreePointTurn>(xml_tag_name, action_name, conf)
{
  float forward_dist, backward_dist, max_speed, max_steering_angle;

  getInput("forward_dist", forward_dist);
  getInput("backward_dist", backward_dist);
  getInput("max_speed", max_speed);
  getInput("max_steering_angle", max_steering_angle);

  goal_.forward_dist = forward_dist;
  goal_.backward_dist = backward_dist;
  goal_.max_speed = max_speed;
  goal_.max_steering_angle = max_steering_angle;

  goal_.time_allowance.sec = 10;
  goal_.time_allowance.nanosec = 0;
}

void ThreePointTurnAction::on_tick()
{
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ThreePointTurnAction>(name, "three_point_turn", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ThreePointTurnAction>("ThreePointTurn", builder);
}
