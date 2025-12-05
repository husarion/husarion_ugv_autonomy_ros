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

#ifndef HUSARION_UGV_NAVIGATION_HUSARION_UGV_NAVIGATION_MAP_AUTOSAVER_HPP
#define HUSARION_UGV_NAVIGATION_HUSARION_UGV_NAVIGATION_MAP_AUTOSAVER_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/save_map.hpp>

namespace husarion_ugv_navigation
{

constexpr auto SAVE_MAP_CONNECTION_TIMEOUT = std::chrono::seconds(2);
constexpr auto MIN_SAVE_MAP_PERIOD = std::chrono::seconds(5);
constexpr auto DEFAULT_MAP_PERIOD = std::chrono::seconds(10);

using SaveMapReq = nav2_msgs::srv::SaveMap::Request;

class AutosaveMapNode : public rclcpp::Node
{
public:
  AutosaveMapNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::chrono::duration<double> autosave_period_ = DEFAULT_MAP_PERIOD;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::TimerBase::SharedPtr save_map_timer_;

  void SaveMapCB();
  SaveMapReq::SharedPtr CreateSaveMapRequest();
};

}  // namespace husarion_ugv_navigation

#endif  // HUSARION_UGV_NAVIGATION_HUSARION_UGV_NAVIGATION_MAP_AUTOSAVER_HPP
