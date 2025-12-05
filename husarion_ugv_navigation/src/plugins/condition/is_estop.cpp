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

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_navigation/plugins/condition/is_estop.hpp"

namespace husarion_ugv_navigation
{

IsEStop::IsEStop(const std::string & condition_name, const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf), estop_(true), topic_("hardware/e_stop")
{
  getInput("topic", topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() { callback_group_executor_.spin(); });

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&IsEStop::eStopCb, this, std::placeholders::_1), sub_option);
}

IsEStop::~IsEStop()
{
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

BT::NodeStatus IsEStop::tick()
{
  if (estop_) {
    RCLCPP_WARN(node_->get_logger(), "E-stop activated. Halting navigation.");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsEStop::eStopCb(const BoolMsg::SharedPtr msg) { estop_ = msg->data; }

}  // namespace husarion_ugv_navigation

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_navigation::IsEStop>("IsEStop");
}
