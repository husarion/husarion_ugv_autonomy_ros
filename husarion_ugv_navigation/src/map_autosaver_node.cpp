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

#include "husarion_ugv_navigation/map_autosaver_node.hpp"

namespace husarion_ugv_navigation
{

AutosaveMapNode::AutosaveMapNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<double>("autosave_period", MIN_SAVE_MAP_PERIOD.count());
  this->declare_parameter<std::string>("map_directory", "/maps/map");

  double period;
  this->get_parameter("autosave_period", period);
  autosave_period_ = std::chrono::duration<double>(period);

  if (autosave_period_ < MIN_SAVE_MAP_PERIOD) {
    RCLCPP_WARN_STREAM(
      get_logger(), "autosave_period is too short. It will be set to the minimum period of "
                      << MIN_SAVE_MAP_PERIOD.count() << " seconds");
    autosave_period_ = MIN_SAVE_MAP_PERIOD;
  }

  save_map_client_ = this->create_client<nav2_msgs::srv::SaveMap>("map_saver/save_map");
  save_map_timer_ = this->create_wall_timer(
    autosave_period_, std::bind(&AutosaveMapNode::SaveMapCB, this));

  RCLCPP_INFO_STREAM(
    get_logger(), "Map saver initialized with period " << autosave_period_.count() << " seconds");
}

void AutosaveMapNode::SaveMapCB()
{
  if (save_map_client_->wait_for_service(SAVE_MAP_CONNECTION_TIMEOUT)) {
    auto request = CreateSaveMapRequest();
    save_map_client_->async_send_request(request);
  } else {
    RCLCPP_WARN(get_logger(), "map_saver/save_map service unavailable");
  }
}

SaveMapReq::SharedPtr AutosaveMapNode::CreateSaveMapRequest()
{
  std::string ns = this->get_namespace();
  if (ns.back() != '/') {
    ns = ns + "/";
  }

  auto request = std::make_shared<SaveMapReq>();
  request->free_thresh = 0.25;
  request->occupied_thresh = 0.65;
  request->map_topic = ns + std::string("map");
  // Allow dynamically override parameter
  this->get_parameter("map_directory", request->map_url);
  request->map_mode = "trinary";
  request->image_format = "png";

  return request;
}

}  // namespace husarion_ugv_navigation
