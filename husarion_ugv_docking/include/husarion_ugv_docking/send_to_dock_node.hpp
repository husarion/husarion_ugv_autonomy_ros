// Copyright 2025 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_SEND_TO_DOCK_NODE_HPP_
#define HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_SEND_TO_DOCK_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/dock_robot.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace husarion_ugv_docking
{
using SetBoolSrv = std_srvs::srv::SetBool;
using DockRobot = nav2_msgs::action::DockRobot;
using GoalHandleDockRobot = rclcpp_action::ClientGoalHandle<DockRobot>;
using SendGoalOptions = rclcpp_action::Client<DockRobot>::SendGoalOptions;

class SendToDockNode : public rclcpp::Node
{
public:
  explicit SendToDockNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void HandleService(
    const SetBoolSrv::Request::SharedPtr request, SetBoolSrv::Response::SharedPtr response);
  void FeedbackCallback(
    GoalHandleDockRobot::SharedPtr, const DockRobot::Feedback::ConstSharedPtr feedback);
  void ResultCallback(const GoalHandleDockRobot::WrappedResult & result);
  void GoalResponseCallback(GoalHandleDockRobot::SharedPtr goal_handle);
  DockRobot::Goal CreateGoalMsg();
  SendGoalOptions CreateGoalOptions();

  GoalHandleDockRobot::SharedPtr active_goal_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_action_client_;
  rclcpp::Service<SetBoolSrv>::SharedPtr service_;
  std::string dock_type_;
  bool navigate_to_staging_pose_;
  std::string dock_id_;
};
}  // namespace husarion_ugv_docking
#endif  // HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_SEND_TO_DOCK_NODE_HPP_
