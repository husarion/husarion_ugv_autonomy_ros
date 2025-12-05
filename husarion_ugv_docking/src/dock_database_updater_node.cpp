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

#include "husarion_ugv_docking/dock_database_updater_node.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/srv/reload_dock_database.hpp>

namespace husarion_ugv_docking
{

DockDatabaseUpdaterNode::DockDatabaseUpdaterNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::vector<std::string>>("docks", {"main"});
  this->declare_parameter<std::string>("dock_database_filepath", "dock_database.yaml");

  dock_names_ = this->get_parameter("docks").as_string_array();
  filepath_ = this->get_parameter("dock_database_filepath").as_string();

  ClearDatabaseFile();

  for (const auto & dock_name : dock_names_) {
    std::string new_dock_pose_topic_name = dock_name + "/new_dock_pose";

    this->declare_parameter<std::string>(dock_name + ".type", "charging_dock");
    this->declare_parameter<std::vector<double>>(dock_name + ".pose", {0.0, 0.0, 0.0});
    this->declare_parameter<std::string>(dock_name + ".frame", "map");

    std::string dock_type = this->get_parameter(dock_name + ".type").as_string();
    std::vector<double> dock_pose = this->get_parameter(dock_name + ".pose").as_double_array();
    std::string frame = this->get_parameter(dock_name + ".frame").as_string();
    if (dock_pose.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "Invalid pose parameter for dock '%s'", dock_name.c_str());
      throw std::runtime_error("Invalid pose parameter");
    }

    PoseStampedMsg::SharedPtr initial_pose = CreateInitialPose(frame, dock_pose);

    if (!UpdateDatabaseFile(dock_name, dock_type, initial_pose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set initial pose in the dock database file.");
      continue;
    }

    auto sub = this->create_subscription<PoseStampedMsg>(
      new_dock_pose_topic_name, 10,
      [this, dock_name, dock_type](const PoseStampedMsg::SharedPtr msg) {
        PoseCallback(dock_name, dock_type, msg);
      });

    subscriptions_.push_back(sub);
    RCLCPP_INFO(
      this->get_logger(), "Subscribed to pose topic: '%s' for dock type '%s'",
      new_dock_pose_topic_name.c_str(), dock_type.c_str());
  }

  auto qos = rclcpp::QoS(rclcpp::ServicesQoS());

  reload_dock_database_client_ = this->create_client<ReloadDockDatabaseSrv>(
    "docking_server/reload_database", qos, client_cb_group_);

  RCLCPP_INFO(this->get_logger(), "Node started.");
}

void DockDatabaseUpdaterNode::PoseCallback(
  const std::string & dock_name, const std::string & dock_type, const PoseStampedMsg::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received pose.");

  using namespace std::chrono_literals;
  if (!reload_dock_database_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Service not available, updating canceled.");
    return;
  }

  if (!UpdateDatabaseFile(dock_name, dock_type, msg)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update dock database file.");
    return;
  }

  auto request = std::make_shared<ReloadDockDatabaseSrv::Request>();
  request->filepath = filepath_;

  reload_dock_database_client_->async_send_request(request);

  RCLCPP_INFO(this->get_logger(), "Sent request to reload dock database.");
}

YAML::Node DockDatabaseUpdaterNode::UpdateDockDatabase(
  const std::string & dock_name, const std::string & dock_type,
  const PoseStampedMsg::SharedPtr pose)
{
  YAML::Node yaml_file;
  auto yaml_docks = yaml_file["docks"];
  auto yaml_dock = yaml_docks[dock_name];

  yaml_dock["type"] = dock_type;

  tf2::Quaternion q(
    pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z,
    pose->pose.orientation.w);

  double yaw = tf2::getYaw(q);
  std::array<double, 3> pose_yaml = {pose->pose.position.x, pose->pose.position.y, yaw};
  yaml_dock["pose"] = pose_yaml;
  yaml_dock["frame"] = pose->header.frame_id;

  return yaml_file;
}

void DockDatabaseUpdaterNode::ClearDatabaseFile()
{
  std::ofstream fout(filepath_, std::ofstream::out | std::ofstream::trunc);
  if (!fout.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open or create the dock database file: '%s'",
      filepath_.c_str());
    throw std::runtime_error("Failed to open or create the dock database file");
  }
  fout.close();
}

bool DockDatabaseUpdaterNode::UpdateDatabaseFile(
  const std::string & dock_name, const std::string & dock_type,
  const PoseStampedMsg::SharedPtr pose)
{
  try {
    yaml_file_ = UpdateDockDatabase(dock_name, dock_type, pose);

    std::ofstream fout(filepath_);
    if (!fout.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open or create the dock database file: '%s'",
        filepath_.c_str());
      throw std::runtime_error("Failed to open or create the dock database file");
    }

    fout << yaml_file_;
    fout.close();

    RCLCPP_INFO(this->get_logger(), "Dock database file updated: '%s'", filepath_.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while updating dock database file: %s", e.what());
    return false;
  }
}

PoseStampedMsg::SharedPtr DockDatabaseUpdaterNode::CreateInitialPose(
  const std::string & frame, const std::vector<double> & pose_vec)
{
  auto pose_msg = std::make_shared<PoseStampedMsg>();
  pose_msg->header.frame_id = frame;
  pose_msg->header.stamp = this->now();

  pose_msg->pose.position.x = pose_vec[0];
  pose_msg->pose.position.y = pose_vec[1];
  pose_msg->pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, pose_vec[2]);
  pose_msg->pose.orientation.x = q.x();
  pose_msg->pose.orientation.y = q.y();
  pose_msg->pose.orientation.z = q.z();
  pose_msg->pose.orientation.w = q.w();

  return pose_msg;
}
}  // namespace husarion_ugv_docking
