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

#include "husarion_ugv_map_gateway/map_gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <lifecycle_msgs/msg/state.hpp>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/map_image.hpp"

namespace husarion_ugv_map_gateway
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

double StampSeconds(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

double YawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

uint64_t GridHash(const nav_msgs::msg::OccupancyGrid & grid)
{
  const auto & info = grid.info;
  const double meta[] = {double(info.width),       double(info.height),
                         info.resolution,          info.origin.position.x,
                         info.origin.position.y,   info.origin.orientation.z,
                         info.origin.orientation.w};
  uint64_t h = Fnv1a64(meta, sizeof(meta));
  h = Fnv1a64(grid.data.data(), grid.data.size(), h);
  return h;
}

// Whether a grid received while the map topic's publishers were `at_grid` (MapPublisherGid's
// sorted, comma-joined GIDs) can come from one of the publishers `now`: every publisher there now
// was there then. None known then means the grid came within the last GID poll (GidTick fills the
// set in at its end) from a publisher the graph did not show yet.
bool GridFromCurrentPublishers(const std::string & now, const std::string & at_grid)
{
  if (at_grid.empty()) {
    return true;
  }
  std::istringstream in(now);
  std::string gid;
  const std::string within = "," + at_grid + ",";
  while (std::getline(in, gid, ',')) {
    if (within.find("," + gid + ",") == std::string::npos) {
      return false;
    }
  }
  return true;
}

std::chrono::nanoseconds Seconds(double s)
{
  return std::chrono::nanoseconds(static_cast<int64_t>(s * 1e9));
}

}  // namespace

MapGatewayNode::MapGatewayNode(const rclcpp::NodeOptions & options) : Node("map_gateway", options)
{
  map_topic_ = declare_parameter<std::string>("map_topic", "map");
  const auto plan_topic = declare_parameter<std::string>("plan_topic", "plan");
  const auto fix_topic = declare_parameter<std::string>("fix_topic", "gps/fix");
  const auto costmap_topic = declare_parameter<std::string>(
    "local_costmap_topic", "local_costmap/costmap");
  const auto status_topic = declare_parameter<std::string>(
    "nav_status_topic", "navigate_to_pose/_action/status");
  map_frame_ = Frame(declare_parameter<std::string>("map_frame", ""), "map");
  base_frame_ = Frame(declare_parameter<std::string>("base_frame", ""), "base_link");
  antenna_frame_ = declare_parameter<std::string>("antenna_frame", "");
  laser_frame_ = Frame(declare_parameter<std::string>("laser_frame", ""), "os_lidar");
  fromll_name_ = declare_parameter<std::string>("fromll_service", "base/fromLL");
  toll_name_ = declare_parameter<std::string>("toll_service", "base/toLL");
  const auto lifecycle_manager = declare_parameter<std::string>(
    "lifecycle_manager", "lifecycle_manager_navigation");
  const auto collision_monitor = declare_parameter<std::string>(
    "collision_monitor", "collision_monitor");
  const auto save_map_service = declare_parameter<std::string>(
    "map_saver_service", "map_saver/save_map");
  const auto slam_serialize_service = declare_parameter<std::string>(
    "slam_serialize_service", "slam_toolbox/serialize_map");
  maps_directory_ = declare_parameter<std::string>("maps_directory", "/maps");
  data_dir_ = declare_parameter<std::string>("data_dir", "");
  if (data_dir_.empty()) {
    data_dir_ = (fs::path(maps_directory_) / ".map_gateway").string();
  }
  session_path_ = (fs::path(data_dir_) / "session.yaml").string();
  declare_parameter<std::string>("map_file_name", "");

  pose_max_age_s_ = declare_parameter<double>("pose_max_age_s", 1.0);
  fix_tf_timeout_s_ = declare_parameter<double>("fix_tf_timeout_s", 0.2);
  service_timeout_s_ = declare_parameter<double>("service_timeout_s", 1.5);
  lifecycle_timeout_s_ = declare_parameter<double>("lifecycle_timeout_s", 10.0);
  save_timeout_s_ = declare_parameter<double>("save_timeout_s", 30.0);
  plan_tolerance_ = declare_parameter<double>("plan_tolerance", 0.05);
  plan_max_poses_ = declare_parameter<int>("plan_max_poses", 200);
  png_budget_ = declare_parameter<int>("png_budget_bytes", 190000);
  png_cap_ = declare_parameter<int>("png_cap_bytes", 200000);
  png_max_side_ = declare_parameter<int>("png_max_side", 2048);
  state_heartbeat_s_ = declare_parameter<double>("state_heartbeat_s", 1.0);
  const double pose_rate = declare_parameter<double>("pose_rate_hz", 8.0);
  pose_period_s_ = 1.0 / pose_rate;
  const double poll_period = declare_parameter<double>("poll_period_s", 2.0);
  state_spacer_ = Spacer<diagnostic_msgs::msg::DiagnosticStatus>(
    declare_parameter<double>("state_min_gap_s", 0.6));
  plan_spacer_ = Spacer<nav_msgs::msg::Path>(declare_parameter<double>("plan_min_gap_s", 1.25));
  map_spacer_ = Spacer<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>(
    declare_parameter<double>("map_min_gap_s", 5.0));
  // R26: 1 Hz under the lane's max_hz of 2 (§5.1 "below the caps").
  double costmap_rate = declare_parameter<double>("costmap_rate_hz", 1.0);
  if (!(costmap_rate > 0.0) || !std::isfinite(costmap_rate)) {
    RCLCPP_WARN(get_logger(), "costmap_rate_hz must be above 0; using 1.0");
    costmap_rate = 1.0;
  } else if (costmap_rate >= 2.0) {
    RCLCPP_WARN(
      get_logger(), "costmap_rate_hz %.2f is at or over the lane's cap of 2 Hz: samples will drop",
      costmap_rate);
  }
  costmap_gate_ = CostmapGate(
    1.0 / costmap_rate, declare_parameter<double>("costmap_move_tolerance_m", 0.005));
  costmap_tf_wait_s_ = declare_parameter<double>("costmap_tf_wait_s", 0.5);
  costmap_cap_ = declare_parameter<int>("costmap_cap_bytes", 32768);
  costmap_max_side_ = declare_parameter<int>("costmap_max_side", 400);

  AlignParams ap;
  ap.min_pairs = declare_parameter<int>("align.min_pairs", ap.min_pairs);
  ap.min_spread_m = declare_parameter<double>("align.min_spread_m", ap.min_spread_m);
  ap.spread_sigma_factor = declare_parameter<double>(
    "align.spread_sigma_factor", ap.spread_sigma_factor);
  ap.max_rms_m = declare_parameter<double>("align.max_rms_m", ap.max_rms_m);
  ap.max_scale_error = declare_parameter<double>("align.max_scale_error", ap.max_scale_error);
  ap.disagree_factor = declare_parameter<double>("align.disagree_factor", ap.disagree_factor);
  ap.disagree_s = declare_parameter<double>("align.disagree_s", ap.disagree_s);
  ap.worse_s = declare_parameter<double>("align.worse_s", ap.worse_s);
  ap.better_s = declare_parameter<double>("align.better_s", ap.better_s);
  ap.held_after_s = declare_parameter<double>("align.held_after_s", ap.held_after_s);
  ap.held_recover_s = declare_parameter<double>("align.held_recover_s", ap.held_recover_s);
  ap.version_move_m = declare_parameter<double>("align.version_move_m", ap.version_move_m);
  ap.pair_min_distance_m = declare_parameter<double>(
    "align.pair_min_distance_m", ap.pair_min_distance_m);
  ap.floors.plain = declare_parameter<double>("align.err_floor_plain", ap.floors.plain);
  ap.floors.rtk_float = declare_parameter<double>("align.err_floor_rtk_float", ap.floors.rtk_float);
  ap.floors.rtk_fixed = declare_parameter<double>("align.err_floor_rtk_fixed", ap.floors.rtk_fixed);
  aligner_ = std::make_unique<Aligner>(ap);

  EpochTracker::Params ep;
  ep.session_wait_s = declare_parameter<double>("session_wait_s", ep.session_wait_s);
  ep.same_map_wait_s = declare_parameter<double>("same_map_wait_s", ep.same_map_wait_s);
  epoch_ = std::make_unique<EpochTracker>(ep);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  state_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "cockpit/nav/state", latched);
  png_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("cockpit/nav/map/png", latched);
  info_pub_ = create_publisher<nav_msgs::msg::MapMetaData>("cockpit/nav/map/info", latched);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "cockpit/nav/pose", rclcpp::QoS(1).best_effort());
  plan_pub_ = create_publisher<nav_msgs::msg::Path>("cockpit/nav/plan", rclcpp::QoS(1).reliable());
  costmap_png_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "cockpit/nav/costmap/png", rclcpp::QoS(1).best_effort());
  costmap_info_pub_ = create_publisher<nav_msgs::msg::MapMetaData>(
    "cockpit/nav/costmap/info", rclcpp::QoS(1).best_effort());

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { OnMap(m); });
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    plan_topic, rclcpp::QoS(1).reliable(),
    [this](nav_msgs::msg::Path::ConstSharedPtr m) { OnPlan(m); });
  status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
    status_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) { OnGoalStatus(m); });
  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) { OnFix(m); });
  // Best effort matches Nav2's reliable publisher and any other; a lost window is replaced by
  // the next one (Nav2 publishes at 2 Hz, the gateway sends at 1).
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, rclcpp::QoS(1).best_effort(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { OnCostmap(m); });

  is_active_client_ = create_client<Trigger>(lifecycle_manager + "/is_active");
  manage_client_ = create_client<ManageNodes>(lifecycle_manager + "/manage_nodes");
  cm_state_client_ = create_client<GetState>(collision_monitor + "/get_state");
  save_map_client_ = create_client<SaveMap>(save_map_service);
  slam_serialize_client_ = create_client<SerializePoseGraph>(slam_serialize_service);

  pause_srv_ = create_service<Trigger>(
    "cockpit/nav/pause", [this](
                           std::shared_ptr<rclcpp::Service<Trigger>> s,
                           std::shared_ptr<rmw_request_id_t> h, std::shared_ptr<Trigger::Request>) {
      OnLifecycleTrigger(ManageNodes::Request::PAUSE, s, h);
    });
  resume_srv_ = create_service<Trigger>(
    "cockpit/nav/resume",
    [this](
      std::shared_ptr<rclcpp::Service<Trigger>> s, std::shared_ptr<rmw_request_id_t> h,
      std::shared_ptr<Trigger::Request>) {
      OnLifecycleTrigger(ManageNodes::Request::RESUME, s, h);
    });
  serialize_srv_ = create_service<SerializePoseGraph>(
    "~/serialize_map",
    [this](
      std::shared_ptr<rclcpp::Service<SerializePoseGraph>> s, std::shared_ptr<rmw_request_id_t> h,
      std::shared_ptr<SerializePoseGraph::Request> r) { OnSerialize(s, h, r); });

  Start();

  timers_.push_back(create_wall_timer(Seconds(1.0 / pose_rate), [this] { PoseTick(); }));
  timers_.push_back(create_wall_timer(1s, [this] { GidTick(); }));
  timers_.push_back(create_wall_timer(1s, [this] { AlignTick(); }));
  timers_.push_back(create_wall_timer(Seconds(poll_period), [this] { PollTick(); }));
  timers_.push_back(create_wall_timer(50ms, [this] { FlushTick(); }));
  PollTick();

  param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : ps) {
      if (p.get_name() == "map_file_name") {
        pending_map_file_ = p.as_string();
      }
    }
    return result;
  });
}

double MapGatewayNode::Now() { return now().seconds(); }

std::string MapGatewayNode::Frame(const std::string & param, const std::string & fallback)
{
  if (!param.empty()) {
    return param;
  }
  std::string ns = get_namespace();
  while (!ns.empty() && ns.front() == '/') {
    ns.erase(0, 1);
  }
  return ns.empty() ? fallback : ns + "/" + fallback;
}

std::string MapGatewayNode::MapPublisherGid()
{
  const std::string topic = get_node_topics_interface()->resolve_topic_name(map_topic_);
  std::vector<std::string> gids;
  for (const auto & info : get_publishers_info_by_topic(topic)) {
    std::ostringstream hex;
    hex << std::hex;
    for (uint8_t b : info.endpoint_gid()) {
      hex << (b < 16 ? "0" : "") << int(b);
    }
    gids.push_back(hex.str());
  }
  std::sort(gids.begin(), gids.end());
  std::string out;
  for (const auto & g : gids) {
    out += (out.empty() ? "" : ",") + g;
  }
  return out;
}

void MapGatewayNode::Start()
{
  const std::string map_file = get_parameter("map_file_name").as_string();
  loaded_sidecar_.reset();
  if (!map_file.empty()) {
    loaded_sidecar_ = LoadMatchingSidecar(map_file);
    if (!loaded_sidecar_) {
      RCLCPP_WARN(
        get_logger(), "No sidecar matches %s: the map gets a new epoch and reads not_aligned",
        map_file.c_str());
    }
  }
  std::error_code ec;
  fs::create_directories(data_dir_, ec);
  const auto session = ReadSession(session_path_);
  epoch_->Start(map_file, loaded_sidecar_, session, Now());
  if (epoch_->decided()) {
    OnEpochChanged(true);  // no older epoch in this process: any grid is this one's
  } else {
    RCLCPP_INFO(get_logger(), "Waiting for the map publisher to match the session file");
  }
}

void MapGatewayNode::OnEpochChanged(bool grid_belongs)
{
  const auto why = epoch_->why();
  if (auto restored = epoch_->TakeRestored()) {
    aligner_->Restore(restored->word, restored->alignment, restored->version);
  } else if (
    why == EpochTracker::Why::kSidecar ||
    (why == EpochTracker::Why::kMapFileChanged && epoch_->on_saved_pair())) {
    if (loaded_sidecar_ && loaded_sidecar_->alignment) {
      aligner_->Load(*loaded_sidecar_->alignment);
    } else {
      aligner_->Reset();
    }
  } else {
    aligner_->Reset();
  }
  sidecar_target_ = epoch_->on_saved_pair() ? epoch_->map_file() : "";
  // The page clears its picture on a new epoch and keeps one only when the state's map_stamp
  // names it, so a new epoch's map_stamp names nothing (0) until one of its own grids is sent, and
  // a grid the spacer still holds is the old epoch's and is dropped. map_server publishes once:
  // when the last grid received is the new epoch's own, it is offered again rather than waiting
  // for the next one. When it is the old map's, nothing is sent until the next grid, which is
  // then sent whatever its hash.
  map_spacer_.Drop();
  map_stamp_.reset();
  have_map_hash_ = false;
  epoch_first_grid_.reset();
  if (last_grid_ && grid_belongs) {
    map_hash_ = GridHash(*last_grid_);
    have_map_hash_ = true;
    epoch_first_grid_ = map_hash_;
    if (auto now_grid = map_spacer_.Offer(last_grid_, Now())) {
      PublishMap(**now_grid);
    }
  }
  last_version_ = aligner_->version();
  last_word_ = aligner_->word();
  RCLCPP_INFO(
    get_logger(), "Map epoch %s (%s), alignment %s", epoch_->epoch().c_str(), EpochWhyName(why),
    WordName(aligner_->word()));
  UpdateLLServices();
  WriteSessionIfChanged(true);
}

void MapGatewayNode::WriteSessionIfChanged(bool force)
{
  if (!epoch_->decided()) {
    return;
  }
  Session s;
  s.epoch = epoch_->epoch();
  s.gid = epoch_->gid();
  s.word = aligner_->word();
  s.version = aligner_->version();
  s.alignment = aligner_->alignment();
  // The numbers are written with the version: refinements that do not move the version do not
  // cost a write.
  std::ostringstream key;
  key << s.epoch << "|" << s.gid << "|" << WordName(s.word) << "|" << s.version;
  if (!force && key.str() == written_session_) {
    return;
  }
  if (WriteSession(session_path_, s)) {
    written_session_ = key.str();
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Cannot write the session file %s", session_path_.c_str());
  }
}

void MapGatewayNode::RewriteSidecarAlignment()
{
  if (sidecar_target_.empty() || !aligner_->alignment()) {
    return;
  }
  const std::string path = SidecarPath(sidecar_target_);
  auto sidecar = ReadSidecar(path);
  if (!sidecar || sidecar->epoch != epoch_->epoch()) {
    return;
  }
  sidecar->alignment = aligner_->alignment();
  if (WriteSidecar(path, *sidecar)) {
    RCLCPP_INFO(
      get_logger(), "Alignment version %u written to %s", aligner_->version(), path.c_str());
  }
}

void MapGatewayNode::UpdateLLServices()
{
  const bool serve = WordServesLL(aligner_->word()) && aligner_->alignment().has_value();
  if (serve && !fromll_srv_) {
    fromll_srv_ = create_service<FromLL>(
      fromll_name_,
      [this](const std::shared_ptr<FromLL::Request> req, std::shared_ptr<FromLL::Response> res) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        res->map_point.x = res->map_point.y = nan;
        res->map_point.z = 0.0;
        const auto & a = aligner_->alignment();
        if (!a || !WordServesLL(aligner_->word())) {
          return;
        }
        if (const auto p = LatLonToMap(a->geo, {req->ll_point.latitude, req->ll_point.longitude})) {
          res->map_point.x = p->e;
          res->map_point.y = p->n;
        }
      });
    toll_srv_ = create_service<ToLL>(
      toll_name_,
      [this](const std::shared_ptr<ToLL::Request> req, std::shared_ptr<ToLL::Response> res) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        res->ll_point.latitude = res->ll_point.longitude = nan;
        res->ll_point.altitude = 0.0;
        const auto & a = aligner_->alignment();
        if (
          !a || !WordServesLL(aligner_->word()) || !AllFinite(req->map_point.x, req->map_point.y)) {
          return;
        }
        const LatLon ll = MapToLatLon(a->geo, req->map_point.x, req->map_point.y);
        res->ll_point.latitude = ll.lat;
        res->ll_point.longitude = ll.lon;
      });
    RCLCPP_INFO(get_logger(), "Serving %s and %s", fromll_name_.c_str(), toll_name_.c_str());
  } else if (!serve && fromll_srv_) {
    fromll_srv_.reset();
    toll_srv_.reset();
    RCLCPP_INFO(
      get_logger(), "Not serving %s and %s: no usable alignment", fromll_name_.c_str(),
      toll_name_.c_str());
  }
}

void MapGatewayNode::OnMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  const uint64_t hash = GridHash(*msg);
  const double now = Now();
  // A restarted publisher's latched grid arrives at discovery, before the 1 s GID poll: look at
  // the publisher first, so this grid is judged as the new publisher's (same map or not). It is
  // the last grid before the check, so a new epoch shows this grid, not the old publisher's.
  last_grid_ = msg;
  last_grid_gids_ = MapPublisherGid();
  if (epoch_->OnPublisher(last_grid_gids_, now)) {
    OnEpochChanged(true);
  }
  if (epoch_->OnGrid(hash, now)) {
    OnEpochChanged(true);  // this grid decided it
  }
  if (!epoch_first_grid_) {
    epoch_first_grid_ = hash;
  }
  const auto & info = msg->info;
  if (AllFinite(info.origin.position.x, info.origin.position.y, info.resolution)) {
    aligner_->SetMapBounds(
      info.origin.position.x, info.origin.position.y,
      info.origin.position.x + info.width * info.resolution,
      info.origin.position.y + info.height * info.resolution);
  }
  if (have_map_hash_ && hash == map_hash_) {
    return;  // slam_toolbox republished the same grid
  }
  map_hash_ = hash;
  have_map_hash_ = true;
  if (auto now_grid = map_spacer_.Offer(msg, now)) {
    PublishMap(**now_grid);
  }
}

void MapGatewayNode::PublishMap(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (!epoch_->decided()) {
    return;
  }
  const auto & info = grid.info;
  if (
    !AllFinite(
      info.resolution, info.origin.position.x, info.origin.position.y, info.origin.position.z,
      info.origin.orientation.x, info.origin.orientation.y, info.origin.orientation.z,
      info.origin.orientation.w) ||
    info.resolution <= 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "A map with non-finite metadata");
    return;
  }
  const GreyGrid grey = ClassifyGrid(grid.data, info.width, info.height, info.resolution);
  const auto png = EncodeWithinBudget(grey, png_budget_, png_max_side_, png_cap_);
  if (!png) {
    RCLCPP_WARN(get_logger(), "The map does not fit the PNG budget at any resolution");
    return;
  }
  sensor_msgs::msg::CompressedImage image;
  image.header.stamp = grid.header.stamp;
  image.header.frame_id = grid.header.frame_id.empty() ? map_frame_ : grid.header.frame_id;
  image.format = "png";
  image.data.assign(png->png.begin(), png->png.end());

  nav_msgs::msg::MapMetaData out;
  out.map_load_time = grid.header.stamp;  // pairs the info with the PNG (§5.1)
  out.resolution = static_cast<float>(png->resolution);
  out.width = png->width;
  out.height = png->height;
  out.origin = info.origin;

  png_pub_->publish(image);
  info_pub_->publish(out);
  map_stamp_ = StampSeconds(grid.header.stamp);
  if (png->halvings > 0) {
    RCLCPP_INFO(
      get_logger(), "Map sent at %.3f m/cell (%u x %u, %zu B) after %d halvings", png->resolution,
      png->width, png->height, png->png.size(), png->halvings);
  }
}

void MapGatewayNode::OnPlan(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  std::vector<Point2> points;
  points.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    const auto & q = p.pose.orientation;
    if (!AllFinite(p.pose.position.x, p.pose.position.y, p.pose.position.z, q.x, q.y, q.z, q.w)) {
      return;  // a plan with a non-finite value is not published (§5.1)
    }
    points.push_back({p.pose.position.x, p.pose.position.y});
  }
  nav_msgs::msg::Path out;
  out.header = msg->header;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = map_frame_;
  }
  for (size_t i : SimplifyPath(points, plan_tolerance_, plan_max_poses_)) {
    geometry_msgs::msg::PoseStamped pose;  // per-pose headers left empty
    pose.pose = msg->poses[i].pose;
    out.poses.push_back(pose);
  }
  if (auto now_plan = plan_spacer_.Offer(out, Now())) {
    plan_pub_->publish(*now_plan);
  }
}

void MapGatewayNode::OnGoalStatus(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
{
  bool active = false;
  for (const auto & s : msg->status_list) {
    active = active || s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
             s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
             s.status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
  }
  if (goal_active_ && !active) {
    nav_msgs::msg::Path empty;  // one empty path when the goal ends (§5.1)
    empty.header.frame_id = map_frame_;
    empty.header.stamp = now();
    if (auto now_plan = plan_spacer_.Offer(empty, Now())) {
      plan_pub_->publish(*now_plan);
    }
  }
  goal_active_ = active;
}

void MapGatewayNode::OnFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  const double now = Now();
  clock_offset_.Add(now, StampSeconds(msg->header.stamp));
  if (!UsableFix(msg->status.status, msg->latitude, msg->longitude)) {
    return;
  }
  FixInput in;
  in.t = now;
  in.ll = {msg->latitude, msg->longitude};
  in.quality = ClassifyFix(
    msg->status.status, msg->position_covariance_type, msg->position_covariance[0],
    msg->position_covariance[4], FixClassParams());
  const std::string antenna = antenna_frame_.empty() ? base_frame_ : antenna_frame_;
  for (const std::string & frame : {antenna, base_frame_}) {
    try {
      // The fix can arrive before the transform for its stamp: wait a little for it (the
      // listener has its own thread).
      const auto t = tf_buffer_->lookupTransform(
        map_frame_, frame, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(fix_tf_timeout_s_));
      in.has_map = AllFinite(t.transform.translation.x, t.transform.translation.y);
      in.x = t.transform.translation.x;
      in.y = t.transform.translation.y;
      break;
    } catch (const tf2::TransformException & e) {
      if (frame == antenna && antenna != base_frame_ && !antenna_warned_) {
        antenna_warned_ = true;
        RCLCPP_WARN(
          get_logger(),
          "No transform to the antenna frame %s; using %s, so the antenna's lever arm is an "
          "error the fit absorbs only partly",
          antenna.c_str(), base_frame_.c_str());
      }
      if (antenna == base_frame_) {
        break;
      }
    }
  }
  aligner_->AddFix(in);
}

void MapGatewayNode::OnCostmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  pending_costmap_ = msg;  // only the newest waits
  pending_costmap_since_ = Now();
}

void MapGatewayNode::CostmapTick(double now)
{
  if (!pending_costmap_ || !costmap_gate_.Due(now)) {
    return;
  }
  const auto grid = pending_costmap_;
  const auto & info = grid->info;
  const auto & o = info.origin;
  if (
    !AllFinite(
      info.resolution, o.position.x, o.position.y, o.orientation.x, o.orientation.y,
      o.orientation.z, o.orientation.w) ||
    info.resolution <= 0.0 || grid->header.frame_id.empty()) {
    pending_costmap_.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "A local costmap with no frame or non-finite metadata");
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    // The window is in odom: place it in the map frame as it was at the grid's stamp. Nav2
    // stamps it about now, so the transform may come a little later: retry on the next ticks.
    t = tf_buffer_->lookupTransform(
      map_frame_, grid->header.frame_id, rclcpp::Time(grid->header.stamp),
      rclcpp::Duration::from_seconds(0.0));
  } catch (const tf2::TransformException & e) {
    if (now - pending_costmap_since_ >= costmap_tf_wait_s_) {
      pending_costmap_.reset();
      ++costmap_no_tf_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No transform %s -> %s at the local costmap's stamp, so it is not sent (%zu so far): %s",
        grid->header.frame_id.c_str(), map_frame_.c_str(), costmap_no_tf_, e.what());
    }
    return;
  }
  pending_costmap_.reset();
  const Pose2 frame{
    t.transform.translation.x, t.transform.translation.y, YawOf(t.transform.rotation)};
  const Pose2 origin = Compose(frame, {o.position.x, o.position.y, YawOf(o.orientation)});
  if (!AllFinite(origin.x, origin.y, origin.yaw)) {
    return;  // every float published is finite (§5.1)
  }
  const double meta[] = {double(info.width), double(info.height), info.resolution};
  const uint64_t hash = Fnv1a64(grid->data.data(), grid->data.size(), Fnv1a64(meta, sizeof(meta)));
  const double extent = std::hypot(double(info.width), double(info.height)) * info.resolution;
  if (!costmap_gate_.Changed(hash, origin, extent)) {
    return;
  }
  CostGrid cells;
  cells.width = info.width;
  cells.height = info.height;
  cells.resolution = info.resolution;
  cells.cells = grid->data;
  const auto png = EncodeCostmap(cells, costmap_max_side_, costmap_cap_);
  if (!png) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "A local costmap that fits no PNG under %zu B",
      costmap_cap_);
    return;
  }
  sensor_msgs::msg::CompressedImage image;
  image.header.stamp = grid->header.stamp;
  image.header.frame_id = map_frame_;
  image.format = "png";
  image.data.assign(png->png.begin(), png->png.end());

  nav_msgs::msg::MapMetaData out;
  out.map_load_time = grid->header.stamp;  // pairs the info with the PNG (§5.1)
  out.resolution = static_cast<float>(png->resolution);
  out.width = png->width;
  out.height = png->height;
  out.origin.position.x = origin.x;
  out.origin.position.y = origin.y;
  out.origin.orientation.z = std::sin(origin.yaw / 2.0);
  out.origin.orientation.w = std::cos(origin.yaw / 2.0);

  costmap_png_pub_->publish(image);
  costmap_info_pub_->publish(out);
  costmap_gate_.Sent(now, hash, origin);
}

void MapGatewayNode::PoseTick()
{
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    const double age = Now() - StampSeconds(t.header.stamp);
    const auto & tr = t.transform.translation;
    const auto & q = t.transform.rotation;
    if (age > pose_max_age_s_ || !AllFinite(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)) {
      pose_ok_ = false;
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = t.header.stamp;
    pose.pose.position.x = tr.x;
    pose.pose.position.y = tr.y;
    pose.pose.position.z = tr.z;
    pose.pose.orientation = q;
    // Never two poses closer than 0.9 periods: the lane's max_hz drops the second (§5.1).
    const double now = Now();
    if (!last_pose_sent_ || now - *last_pose_sent_ >= 0.9 * pose_period_s_) {
      pose_pub_->publish(pose);
      last_pose_sent_ = now;
    }
    pose_ok_ = true;
  } catch (const tf2::TransformException &) {
    pose_ok_ = false;
  }
  if (!laser_) {
    try {
      const auto t = tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero);
      const auto & tr = t.transform.translation;
      const double yaw = YawOf(t.transform.rotation);
      if (AllFinite(tr.x, tr.y, yaw)) {
        laser_ = std::array<double, 3>{tr.x, tr.y, yaw};
      }
    } catch (const tf2::TransformException &) {
    }
  }
}

void MapGatewayNode::GidTick()
{
  if (pending_map_file_) {
    const std::string map_file = *pending_map_file_;
    pending_map_file_.reset();
    loaded_sidecar_ = map_file.empty() ? std::nullopt : LoadMatchingSidecar(map_file);
    // The new map's grid and the parameter come in either order. The last grid is the new map's
    // when it differs from the first grid of the old epoch: on a saved map (the old
    // map_file_name set) map_server publishes once per map it loads, so another grid since the
    // epoch began is the newly loaded one, and it is sent again under the new epoch. The same
    // grid means the new map's has not come: nothing is sent until it does. On mapping (the old
    // map_file_name empty) slam_toolbox's grid changes by itself and cannot tell: nothing is sent
    // until its next grid. The cost: a map with the very content of the old one, loaded before
    // the parameter, shows no picture until map_server publishes again.
    const bool grid_belongs = !epoch_->map_file().empty() && last_grid_ && epoch_first_grid_ &&
                              GridHash(*last_grid_) != *epoch_first_grid_;
    if (epoch_->OnMapFile(map_file, loaded_sidecar_, Now())) {
      OnEpochChanged(grid_belongs);
    }
  }
  // A new publisher decided here: the GID poll saw it before its grid, or a saved pair's new
  // publisher sent no grid within same_map_wait_s. The last grid is the epoch's own only when
  // every publisher of the new epoch was already there when that grid came (an old publisher
  // that leaves late, after the new one's grid, still lets that grid be sent again). The first
  // decision (a session matched, or its wait ran out) has no older epoch: any grid is its own.
  const bool was_decided = epoch_->decided();
  if (epoch_->OnPublisher(MapPublisherGid(), Now()) || epoch_->OnTick(Now())) {
    OnEpochChanged(
      !was_decided || (last_grid_ && GridFromCurrentPublishers(epoch_->gid(), last_grid_gids_)));
  }
  // A grid that came while the graph showed no publisher: its publishers are the ones seen now,
  // so the unknown lasts one poll and never lets a later publisher change resend it.
  if (last_grid_ && last_grid_gids_.empty()) {
    last_grid_gids_ = MapPublisherGid();
  }
  WriteSessionIfChanged();
}

void MapGatewayNode::AlignTick()
{
  aligner_->Tick(Now());
  if (aligner_->version() != last_version_) {
    if (aligner_->word() == Word::kAligned) {
      RewriteSidecarAlignment();
    }
    last_version_ = aligner_->version();
  }
  if (aligner_->word() != last_word_) {
    RCLCPP_INFO(
      get_logger(), "Alignment %s -> %s", WordName(last_word_), WordName(aligner_->word()));
    last_word_ = aligner_->word();
  }
  UpdateLLServices();
  WriteSessionIfChanged();
}

void MapGatewayNode::PollTick()
{
  if (!poll_pending_is_active_) {
    poll_pending_is_active_ = true;
    CallAsync<Trigger>(
      is_active_client_, std::make_shared<Trigger::Request>(), service_timeout_s_,
      [this](Trigger::Response::SharedPtr res) {
        poll_pending_is_active_ = false;
        nav2_ = !res ? "absent" : (res->success ? "active" : "inactive");
      });
  }
  if (!poll_pending_cm_) {
    poll_pending_cm_ = true;
    CallAsync<GetState>(
      cm_state_client_, std::make_shared<GetState::Request>(), service_timeout_s_,
      [this](GetState::Response::SharedPtr res) {
        poll_pending_cm_ = false;
        collision_monitor_ =
          !res ? "absent"
               : (res->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
                    ? "active"
                    : "inactive");
      });
  }
}

std::vector<std::pair<std::string, std::string>> MapGatewayNode::BuildValues()
{
  const double now = Now();
  StateInputs in;
  in.frame = map_frame_;
  in.epoch = epoch_->epoch();
  in.word = aligner_->word();
  in.version = aligner_->version();
  in.alignment = aligner_->alignment();
  in.err_m = aligner_->err_m();
  in.need_m = aligner_->need_m();
  const RigidFit & fit = aligner_->fit();
  if (fit.valid) {
    in.progress_rms_m = fit.rms;
    in.progress_extent_m = fit.spread;
    in.progress_pairs = fit.inliers;
    in.progress_scale = fit.scale;
  } else {
    in.progress_pairs = static_cast<int>(aligner_->pair_count());
  }
  in.last_fix = aligner_->last_fix_class();
  if (aligner_->last_fix_time()) {
    in.fix_age_s = now - *aligner_->last_fix_time();
  }
  in.mode = epoch_->map_file().empty() ? "mapping" : "amcl";
  in.pose_ok = pose_ok_;
  in.nav2 = nav2_;
  in.collision_monitor = collision_monitor_;
  in.laser = laser_;
  in.map_stamp = map_stamp_;
  in.clock_offset_s = clock_offset_.Value(now);
  return StateValues(in);
}

void MapGatewayNode::PublishState(const std::vector<std::pair<std::string, std::string>> & values)
{
  diagnostic_msgs::msg::DiagnosticStatus msg;
  msg.level = WordIsOk(aligner_->word()) ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                         : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  msg.name = kStateName;
  msg.message = WordName(aligner_->word());
  msg.hardware_id = epoch_->epoch();
  for (const auto & [k, v] : values) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    msg.values.push_back(kv);
  }
  if (auto now_msg = state_spacer_.Offer(msg, Now())) {
    state_pub_->publish(*now_msg);
  }
}

void MapGatewayNode::FlushTick()
{
  const double now = Now();
  CostmapTick(now);
  if (auto grid = map_spacer_.Poll(now)) {
    PublishMap(**grid);
  }
  if (auto plan = plan_spacer_.Poll(now)) {
    plan_pub_->publish(*plan);
  }
  if (!epoch_->decided()) {
    return;  // no state before the epoch is known: a state must always carry a valid epoch
  }
  auto values = BuildValues();
  // A change is a change of anything but the two values that move by themselves; those are
  // refreshed by the heartbeat.
  auto significant = [](std::vector<std::pair<std::string, std::string>> v) {
    v.erase(
      std::remove_if(
        v.begin(), v.end(),
        [](const auto & kv) { return kv.first == "fix_age_s" || kv.first == "clock_offset_s"; }),
      v.end());
    return v;
  };
  const bool changed = significant(values) != significant(last_values_);
  const auto sent = state_spacer_.last_sent();
  const bool heartbeat = !sent || now - *sent >= state_heartbeat_s_;
  if (changed || heartbeat) {
    last_values_ = values;
    PublishState(values);
  }
  if (auto held = state_spacer_.Poll(now)) {
    state_pub_->publish(*held);
  }
}

template <typename SrvT>
void MapGatewayNode::CallAsync(
  typename rclcpp::Client<SrvT>::SharedPtr client, typename SrvT::Request::SharedPtr request,
  double timeout_s, std::function<void(typename SrvT::Response::SharedPtr)> done)
{
  // Forget the timers of calls that ended.
  for (auto it = call_timers_.begin(); it != call_timers_.end();) {
    if (it->first->is_canceled()) {
      it->second->reset();
      it = call_timers_.erase(it);
    } else {
      ++it;
    }
  }
  if (!client->service_is_ready()) {
    done(nullptr);
    return;
  }
  auto finished = std::make_shared<bool>(false);
  auto slot = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  auto pending = client->async_send_request(
    request, [finished, done, slot](typename rclcpp::Client<SrvT>::SharedFuture future) {
      if (*finished) {
        return;
      }
      *finished = true;
      if (*slot) {
        (*slot)->cancel();
      }
      done(future.get());
    });
  const int64_t id = pending.request_id;
  std::weak_ptr<rclcpp::Client<SrvT>> weak_client = client;
  *slot = create_wall_timer(Seconds(timeout_s), [finished, done, slot, weak_client, id]() {
    (*slot)->cancel();
    if (*finished) {
      return;
    }
    *finished = true;
    if (auto c = weak_client.lock()) {
      c->remove_pending_request(id);
    }
    done(nullptr);
  });
  call_timers_.emplace_back(*slot, slot);
}

void MapGatewayNode::OnLifecycleTrigger(
  uint8_t command, std::shared_ptr<rclcpp::Service<Trigger>> service,
  std::shared_ptr<rmw_request_id_t> header)
{
  auto request = std::make_shared<ManageNodes::Request>();
  request->command = command;  // PAUSE (1) or RESUME (2), and nothing else (D47)
  const char * what = command == ManageNodes::Request::PAUSE ? "PAUSE" : "RESUME";
  RCLCPP_INFO(get_logger(), "Sending %s to Nav2's lifecycle manager", what);
  CallAsync<ManageNodes>(
    manage_client_, request, lifecycle_timeout_s_,
    [this, service, header, what](ManageNodes::Response::SharedPtr res) {
      Trigger::Response out;
      out.success = res && res->success;
      out.message = !res ? std::string(what) + ": Nav2's lifecycle manager did not answer"
                         : std::string(what) + (res->success ? ": done" : ": Nav2 refused");
      RCLCPP_INFO(get_logger(), "%s", out.message.c_str());
      service->send_response(*header, out);
    });
}

void MapGatewayNode::OnSerialize(
  std::shared_ptr<rclcpp::Service<SerializePoseGraph>> service,
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<SerializePoseGraph::Request> request)
{
  auto fail = [this, service, header](const std::string & why) {
    RCLCPP_WARN(get_logger(), "serialize_map refused: %s", why.c_str());
    SerializePoseGraph::Response out;
    out.result = SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE;
    service->send_response(*header, out);
  };
  const std::string name = request->filename;
  // The name is confined (§5.5, [06] m14): one path component, under the maps directory.
  if (!ValidSaveName(name)) {
    fail("the name must be 1 to 64 characters of A-Z, a-z, 0-9, _ and -");
    return;
  }
  if (saving_) {
    fail("a save is already running");
    return;
  }
  if (!epoch_->decided()) {
    fail("the map epoch is not known yet");
    return;
  }
  const fs::path dir = fs::path(maps_directory_) / name;
  const std::string map_yaml = (dir / "map.yaml").string();
  if (fs::exists(map_yaml)) {
    fail(map_yaml + " exists; a saved pair is never overwritten");
    return;
  }
  SavedPose pose;
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    pose = {t.transform.translation.x, t.transform.translation.y, YawOf(t.transform.rotation)};
    if (!AllFinite(pose.x, pose.y, pose.yaw)) {
      throw tf2::TransformException("not finite");
    }
  } catch (const tf2::TransformException & e) {
    fail(std::string("no robot pose in the map frame (") + e.what() + ")");
    return;
  }
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    fail("cannot create " + dir.string() + ": " + ec.message());
    return;
  }

  saving_ = true;
  const std::string epoch = epoch_->epoch();
  const bool aligned = WordServesLL(aligner_->word()) && aligner_->alignment().has_value();
  const std::optional<Alignment> alignment = aligned ? aligner_->alignment()
                                                     : std::optional<Alignment>();
  auto save = std::make_shared<SaveMap::Request>();
  save->map_topic = get_node_topics_interface()->resolve_topic_name(map_topic_);
  save->map_url = (dir / "map").string();
  save->image_format = "png";
  save->map_mode = "trinary";
  save->free_thresh = 0.25;
  save->occupied_thresh = 0.65;
  RCLCPP_INFO(get_logger(), "Saving the map pair %s", dir.string().c_str());
  CallAsync<SaveMap>(
    save_map_client_, save, save_timeout_s_,
    [this, service, header, dir, map_yaml, epoch, alignment, pose,
     fail](SaveMap::Response::SharedPtr res) {
      if (!res || !res->result) {
        saving_ = false;
        fail(
          !res ? "map_saver/save_map is not available or did not answer"
               : "map_saver could not save the map");
        return;
      }
      const auto image = MapImagePath(map_yaml);
      const auto digest = image ? Sha256File(*image) : std::nullopt;
      if (!digest) {
        saving_ = false;
        fail("map_saver wrote no image beside " + map_yaml);
        return;
      }
      Sidecar sidecar;
      sidecar.epoch = epoch;
      sidecar.map_sha256 = *digest;
      sidecar.alignment = alignment;
      sidecar.saved_pose = pose;
      if (!WriteSidecar(SidecarPath(map_yaml), sidecar)) {
        saving_ = false;
        fail("cannot write " + SidecarPath(map_yaml));
        return;
      }
      if (epoch == epoch_->epoch()) {
        sidecar_target_ = map_yaml;
      }
      RCLCPP_INFO(get_logger(), "Saved %s with its sidecar", map_yaml.c_str());
      // The pose graph too, for a later slam_toolbox localization mode (nothing loads it yet).
      auto graph = std::make_shared<SerializePoseGraph::Request>();
      graph->filename = (dir / "map").string();
      CallAsync<SerializePoseGraph>(
        slam_serialize_client_, graph, save_timeout_s_,
        [this, service, header](SerializePoseGraph::Response::SharedPtr gres) {
          if (!gres || gres->result != SerializePoseGraph::Response::RESULT_SUCCESS) {
            RCLCPP_WARN(
              get_logger(),
              "slam_toolbox's serialize_map %s; the saved pair is complete without it",
              gres ? "failed" : "is not available");
          }
          saving_ = false;
          SerializePoseGraph::Response out;
          out.result = SerializePoseGraph::Response::RESULT_SUCCESS;
          service->send_response(*header, out);
        });
    });
}

}  // namespace husarion_ugv_map_gateway
