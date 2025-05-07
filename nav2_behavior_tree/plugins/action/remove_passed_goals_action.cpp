// Copyright (c) 2021 Samsung Research America
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
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/validate_messages.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  viapoint_achieved_radius_(0.5),
  costmap_(nullptr)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node_->create_publisher<std_msgs::msg::UInt64>("goal_number",10);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_all();

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap",
    map_qos,
    std::bind(&RemovePassedGoals::incomingMap, this, std::placeholders::_1),sub_option);

  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void RemovePassedGoals::initialize()
{
  getInput("radius", viapoint_achieved_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
    node_, "robot_base_frame", this);

}

void RemovePassedGoals::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map){
  RCLCPP_DEBUG(node_->get_logger(), "aaaaaaaaaaaaaaaaaaaaaaa.");
  if (!nav2_util::validateMsg(*new_map)) {
    RCLCPP_ERROR(node_->get_logger(), "Received map message is malformed. Rejecting.");
    return;
  }
  RCLCPP_DEBUG(node_->get_logger(), "Receve global_costmap with size: %lu.",new_map->data.size());
  costmap_ = new_map;
}


inline BT::NodeStatus RemovePassedGoals::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize(); 
  }

  callback_group_executor_.spin_some();

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, goal_poses[0].header.frame_id, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  auto message = std_msgs::msg::UInt64();
  message.data = goal_poses.size();

  publisher_->publish(message);

  double dist_to_goal;
  while (goal_poses.size() > 1) {
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (dist_to_goal > viapoint_achieved_radius_) {
      break;
    }

    goal_poses.erase(goal_poses.begin());
  }
  bool ret = true;
  if(!goal_poses.empty())
    ret = isGoalInObstacle(goal_poses[0]);
  (void)ret;

  setOutput("output_goals", goal_poses);

  return BT::NodeStatus::SUCCESS;
}
bool 
RemovePassedGoals::isGoalInObstacle(const geometry_msgs::msg::PoseStamped & goal){
  if(!costmap_){
    return true;
  }
  double wx = goal.pose.position.x - costmap_->info.origin.position.x;
  double wy = goal.pose.position.y - costmap_->info.origin.position.y;
  if(wx<0||wy<0){
    RCLCPP_ERROR(node_->get_logger(),"goal out of map: goal(%.3lf,%.3lf)",goal.pose.position.x,goal.pose.position.y);
    return true;
  }
  uint32_t mx = (uint32_t)(wx/((double)costmap_->info.resolution));
  uint32_t my = (uint32_t)(wy/((double)costmap_->info.resolution));
  if(mx>=costmap_->info.width||my>=costmap_->info.height){
    RCLCPP_ERROR(node_->get_logger(),"goal out of map: goal(%.3lf,%.3lf)",goal.pose.position.x,goal.pose.position.y);
    return true;
  }

  std::vector<std::vector<int8_t>> near_costs(2*RADIUS+1,std::vector<int8_t>(2*RADIUS+1,-1));
  if(mx<=RADIUS||mx>=UINT32_MAX-RADIUS||my<=RADIUS||my>=UINT32_MAX-RADIUS) return true;
  // 左上到右下遍历
  for(uint32_t i=0;i<2*RADIUS+1;i++){
    for(uint32_t j=0;j<2*RADIUS+1;j++){
      if(mx+j-2>=costmap_->info.width||my+2*RADIUS-i-2>=costmap_->info.height){
        near_costs[i][j] = -1;
        continue;
      }
      near_costs[i][j] = costmap_->data[(my+2*RADIUS-i-2)*costmap_->info.width+(mx+j-2)];
    }
  }
  // 左上到右下遍历
  std::string data_to_print{"\n"};
  for(uint32_t i=0;i<2*RADIUS+1;i++){
    for(uint32_t j=0;j<2*RADIUS+1;j++){
      data_to_print += int8ToFixedString(near_costs[i][j]);
    }
    data_to_print += '\n';
  }
  RCLCPP_ERROR(node_->get_logger(),data_to_print.c_str());
  costmap_ = nullptr;
  return true;
}
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
