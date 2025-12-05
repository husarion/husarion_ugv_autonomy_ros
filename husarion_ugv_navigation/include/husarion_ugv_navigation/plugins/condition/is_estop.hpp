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

#ifndef HUSARION_UGV_NAVIGATION_HUSARION_UGV_NAVIGATION_PLUGINS_CONDITION_IS_ESTOP_HPP_
#define HUSARION_UGV_NAVIGATION_HUSARION_UGV_NAVIGATION_PLUGINS_CONDITION_IS_ESTOP_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

namespace husarion_ugv_navigation
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class IsEStop : public BT::ConditionNode
{
  using BoolMsg = std_msgs::msg::Bool;

public:
  /**
   * @brief A constructor for husarion_ugv_navigation::IsEStop
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsEStop(const std::string & condition_name, const BT::NodeConfig & conf);

  IsEStop() = delete;

  ~IsEStop() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Store value of e-stop status
   */
  void eStopCb(const BoolMsg::SharedPtr msg);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "hardware/e_stop	", "The Bool type topic contains e-stop status"),
    };
  }

private:
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  rclcpp::Subscription<BoolMsg>::SharedPtr estop_sub_;

  bool estop_;
  std::string topic_;
};

}  // namespace husarion_ugv_navigation

#endif  // HUSARION_UGV_NAVIGATION__PLUGINS__CONDITION__IS_ESTOP_HPP_
