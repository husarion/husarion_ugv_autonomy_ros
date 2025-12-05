// Copyright 2025 Husarion Sp. z o.o.
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

#include "husarion_ugv_docking/send_to_dock_node.hpp"

namespace husarion_ugv_docking
{

SendToDockNode::SendToDockNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  this->declare_parameter("dock_type", "charging_dock");
  this->declare_parameter("navigate_to_staging_pose", true);
  this->declare_parameter("dock_id", "main");

  this->get_parameter("dock_type", dock_type_);
  this->get_parameter("navigate_to_staging_pose", navigate_to_staging_pose_);
  this->get_parameter("dock_id", dock_id_);

  dock_action_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

  service_ = this->create_service<SetBoolSrv>(
    "send_robot_to_dock",
    std::bind(&SendToDockNode::HandleService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Service server 'send_robot_to_dock' ready");
}

void SendToDockNode::FeedbackCallback(
  GoalHandleDockRobot::SharedPtr, const DockRobot::Feedback::ConstSharedPtr feedback)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "Feedback received: dock state = %d",
    feedback->state);
}

void SendToDockNode::ResultCallback(const GoalHandleDockRobot::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Docking succeeded! Robot is charging.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Docking aborted!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Docking canceled by user.");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
      break;
  }
  active_goal_.reset();
}

void SendToDockNode::GoalResponseCallback(GoalHandleDockRobot::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by the server");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the server");
  active_goal_ = goal_handle;
}

DockRobot::Goal SendToDockNode::CreateGoalMsg()
{
  auto goal_msg = DockRobot::Goal();
  goal_msg.dock_type = dock_type_;
  goal_msg.navigate_to_staging_pose = navigate_to_staging_pose_;
  goal_msg.dock_id = dock_id_;

  return goal_msg;
}

SendGoalOptions SendToDockNode::CreateGoalOptions()
{
  auto goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();

  goal_options.feedback_callback = std::bind(
    &SendToDockNode::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  goal_options.result_callback = std::bind(
    &SendToDockNode::ResultCallback, this, std::placeholders::_1);

  goal_options.goal_response_callback = std::bind(
    &SendToDockNode::GoalResponseCallback, this, std::placeholders::_1);

  return goal_options;
}

void SendToDockNode::HandleService(
  const SetBoolSrv::Request::SharedPtr request, SetBoolSrv::Response::SharedPtr response)
{
  if (!this->dock_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Docking action server is not available after waiting");
    response->success = false;
    response->message = "Docking action server is not available";
    return;
  }

  if (request->data) {
    if (active_goal_) {
      response->success = true;  // Set to true after receiving every request for
                                 // proper functioning of Vizanti buttons
      response->message = "Docking goal already exists.";
      RCLCPP_WARN(this->get_logger(), "Docking goal already exists.");
      return;
    }

    auto goal_msg = this->CreateGoalMsg();
    auto goal_options = this->CreateGoalOptions();
    dock_action_client_->async_send_goal(goal_msg, goal_options);
    response->success = true;
    response->message = "New docking goal request sent.";
    RCLCPP_INFO(this->get_logger(), "New docking goal request sent.");
    return;
  }

  if (active_goal_) {
    dock_action_client_->async_cancel_all_goals();
    response->success = true;
    response->message = "Docking canceled.";
    RCLCPP_INFO(this->get_logger(), "Docking canceled.");
    return;
  }
  response->success = true;
  response->message = "No active docking goal to cancel.";
  RCLCPP_WARN(this->get_logger(), "No active docking goal to cancel.");
}
}  // namespace husarion_ugv_docking
