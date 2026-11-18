// Copyright 2026 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_MAP_GATEWAY_MAP_GATEWAY_NODE_HPP_
#define HUSARION_UGV_MAP_GATEWAY_MAP_GATEWAY_NODE_HPP_

#include <array>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "husarion_ugv_map_gateway/align.hpp"
#include "husarion_ugv_map_gateway/epoch.hpp"
#include "husarion_ugv_map_gateway/files.hpp"
#include "husarion_ugv_map_gateway/products.hpp"
#include "husarion_ugv_map_gateway/state.hpp"

namespace husarion_ugv_map_gateway
{

// The fleet-side map gateway (SPEC-MAP-NAV-2026-11 §5). It reads Nav2's and slam_toolbox's raw
// topics and publishes small products for the Cockpit. It never publishes a velocity and never
// sends a goal; the only commands it sends are PAUSE and RESUME to Nav2's lifecycle manager on
// its two triggers (D47), and the save calls of serialize_map.
class MapGatewayNode : public rclcpp::Node
{
public:
  explicit MapGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;
  using ManageNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using GetState = lifecycle_msgs::srv::GetState;
  using SaveMap = nav2_msgs::srv::SaveMap;
  using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;
  using FromLL = robot_localization::srv::FromLL;
  using ToLL = robot_localization::srv::ToLL;

  double Now();
  std::string Frame(const std::string & param, const std::string & fallback);
  std::string MapPublisherGid();

  void Start();
  // `grid_belongs`: the last grid received is the new epoch's own, so it is sent again; otherwise
  // the epoch shows no picture until its first grid comes.
  void OnEpochChanged(bool grid_belongs);
  void WriteSessionIfChanged(bool force = false);
  void RewriteSidecarAlignment();
  void UpdateLLServices();

  void OnMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void PublishMap(const nav_msgs::msg::OccupancyGrid & grid);
  void OnPlan(nav_msgs::msg::Path::ConstSharedPtr msg);
  void OnGoalStatus(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg);
  void OnFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  void PoseTick();
  void GidTick();
  void AlignTick();
  void PollTick();
  void FlushTick();

  std::vector<std::pair<std::string, std::string>> BuildValues();
  void PublishState(const std::vector<std::pair<std::string, std::string>> & values);

  void OnLifecycleTrigger(
    uint8_t command, std::shared_ptr<rclcpp::Service<Trigger>> service,
    std::shared_ptr<rmw_request_id_t> header);
  void OnSerialize(
    std::shared_ptr<rclcpp::Service<SerializePoseGraph>> service,
    std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<SerializePoseGraph::Request> request);

  // A service call that always ends: `done` gets the response, or nullptr when the service is not
  // there or does not answer within `timeout_s`.
  template <typename SrvT>
  void CallAsync(
    typename rclcpp::Client<SrvT>::SharedPtr client, typename SrvT::Request::SharedPtr request,
    double timeout_s, std::function<void(typename SrvT::Response::SharedPtr)> done);

  // Parameters.
  std::string map_frame_, base_frame_, antenna_frame_, laser_frame_;
  std::string map_topic_;
  std::string maps_directory_, data_dir_, session_path_;
  double pose_max_age_s_ = 1.0;
  double fix_tf_timeout_s_ = 0.2;
  double service_timeout_s_ = 1.5;
  double lifecycle_timeout_s_ = 10.0;
  double save_timeout_s_ = 30.0;
  double plan_tolerance_ = 0.05;
  size_t plan_max_poses_ = 200;
  size_t png_budget_ = 190000, png_cap_ = 200000;
  uint32_t png_max_side_ = 2048;
  double state_heartbeat_s_ = 1.0;

  // State.
  std::unique_ptr<Aligner> aligner_;
  std::unique_ptr<EpochTracker> epoch_;
  std::optional<Sidecar> loaded_sidecar_;
  std::string sidecar_target_;  // the pair the gateway runs on, or last saved this session
  std::string written_session_;
  uint32_t last_version_ = 0;
  Word last_word_ = Word::kNotAligned;

  uint64_t map_hash_ = 0;
  bool have_map_hash_ = false;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_grid_;
  std::string last_grid_gids_;                // the map topic's publishers when the last grid came
  std::optional<uint64_t> epoch_first_grid_;  // the hash of the first grid of this epoch
  double pose_period_s_ = 0.125;
  std::optional<double> last_pose_sent_;
  std::optional<double> map_stamp_;
  bool pose_ok_ = false;
  std::optional<std::array<double, 3>> laser_;
  std::string nav2_ = "absent", collision_monitor_ = "absent";
  bool goal_active_ = false;
  bool antenna_warned_ = false;
  ClockOffset clock_offset_;

  Spacer<diagnostic_msgs::msg::DiagnosticStatus> state_spacer_{0.6};
  Spacer<nav_msgs::msg::Path> plan_spacer_{1.25};
  Spacer<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> map_spacer_{5.0};
  std::vector<std::pair<std::string, std::string>> last_values_;
  std::optional<double> last_state_sent_;

  // ROS.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr png_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_, resume_srv_;
  rclcpp::Service<SerializePoseGraph>::SharedPtr serialize_srv_;
  rclcpp::Service<FromLL>::SharedPtr fromll_srv_;
  rclcpp::Service<ToLL>::SharedPtr toll_srv_;
  std::string fromll_name_, toll_name_;
  rclcpp::Client<Trigger>::SharedPtr is_active_client_;
  rclcpp::Client<GetState>::SharedPtr cm_state_client_;
  rclcpp::Client<ManageNodes>::SharedPtr manage_client_;
  rclcpp::Client<SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<SerializePoseGraph>::SharedPtr slam_serialize_client_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::list<std::pair<rclcpp::TimerBase::SharedPtr, std::shared_ptr<rclcpp::TimerBase::SharedPtr>>>
    call_timers_;
  bool poll_pending_is_active_ = false, poll_pending_cm_ = false;
  bool saving_ = false;
  std::optional<std::string> pending_map_file_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_MAP_GATEWAY_NODE_HPP_
