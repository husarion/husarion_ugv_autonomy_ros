// Copyright 2024 Husarion sp. z o.o.
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

#include "husarion_ugv_docking/docking_manager_node.hpp"
#include "husarion_ugv_docking/docking_manager_parameters.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include <husarion_ugv_manager/behavior_tree_manager.hpp>
#include <husarion_ugv_manager/behavior_tree_utils.hpp>
#include <husarion_ugv_utils/moving_average.hpp>

namespace husarion_ugv_docking
{

DockingManagerNode::DockingManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->param_listener_ =
    std::make_shared<docking_manager::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  const std::map<std::string, std::any> docking_bb = {
    {"GAMEPAD_DOCKING_SEQUENCE", this->params_.gamepad_docking_sequence},
    {"GAMEPAD_UNDOCKING_SEQUENCE", this->params_.gamepad_undocking_sequence},
  };
  const int bt_server_port = this->params_.bt_server_port;

  docking_tree_manager_ = std::make_unique<husarion_ugv_manager::BehaviorTreeManager>(
    "Docking", docking_bb, bt_server_port);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DockingManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  docking_tree_manager_->Initialize(factory_);

  using namespace std::placeholders;

  const auto timer_freq = this->params_.timer_frequency;
  const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

  docking_tree_timer_ = this->create_wall_timer(
    timer_period, std::bind(&DockingManagerNode::TimerCB, this));
}

void DockingManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->params_.bt_project_path;
  const auto plugin_libs = this->params_.plugin_libs;
  const auto ros_plugin_libs = this->params_.ros_plugin_libs;
  const auto service_availability_timeout = this->params_.ros_communication_timeout.availability;
  const auto service_response_timeout = this->params_.ros_communication_timeout.response;

  BT::RosNodeParams params;
  params.nh = this->shared_from_this();
  auto wait_for_server_timeout_s = std::chrono::duration<double>(service_availability_timeout);
  params.wait_for_server_timeout =
    std::chrono::duration_cast<std::chrono::milliseconds>(wait_for_server_timeout_s);
  auto server_timeout_s = std::chrono::duration<double>(service_response_timeout);
  params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(server_timeout_s);

  husarion_ugv_manager::behavior_tree_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

  RCLCPP_INFO_STREAM(
    this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

void DockingManagerNode::TimerCB()
{
  docking_tree_manager_->TickOnce();

  if (docking_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Docking behavior tree returned FAILURE status");
  }
}

}  // namespace husarion_ugv_docking
