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

#ifndef HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_POSE_CONVERTER_NODE_HPP_
#define HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_POSE_CONVERTER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/srv/reload_dock_database.hpp>

namespace husarion_ugv_docking
{

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using ReloadDockDatabaseSrv = nav2_msgs::srv::ReloadDockDatabase;

class DockDatabaseUpdaterNode : public rclcpp::Node
{
public:
  DockDatabaseUpdaterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void PoseCallback(
    const std::string & dock_name, const std::string & dock_type,
    const PoseStampedMsg::SharedPtr msg);

  void ClearDatabaseFile();

  bool UpdateDatabaseFile(
    const std::string & dock_name, const std::string & dock_type,
    const PoseStampedMsg::SharedPtr pose);
  YAML::Node UpdateDockDatabase(
    const std::string & dock_name, const std::string & dock_type,
    const PoseStampedMsg::SharedPtr pose);
  PoseStampedMsg::SharedPtr CreateInitialPose(
    const std::string & frame, const std::vector<double> & pose_vec);
  std::vector<rclcpp::Subscription<PoseStampedMsg>::SharedPtr> subscriptions_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Client<ReloadDockDatabaseSrv>::SharedPtr reload_dock_database_client_;

  YAML::Node yaml_file_;
  std::vector<std::string> dock_names_;
  std::string filepath_;
};

}  // namespace husarion_ugv_docking

#endif  // HUSARION_UGV_DOCKING_HUSARION_UGV_DOCKING_POSE_CONVERTER_NODE_HPP_
