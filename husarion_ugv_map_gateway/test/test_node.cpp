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

// The gateway node over the real middleware (§11.2): `collision_monitor` reads active, inactive and
// absent from the node's lifecycle state; the pause trigger sends PAUSE (1) and the resume trigger
// RESUME (2), nothing else, each answering with Nav2's success; the epoch on a changed publisher
// GID, restored when the gateway restarts alone, the sidecar's across a map_server restart on the
// same map; fromLL and toLL not advertised without an alignment; the save step and its name rule;
// the map, pose and plan products; clock_offset_s; the spacing of `state`.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
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

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/files.hpp"
#include "husarion_ugv_map_gateway/geo.hpp"
#include "husarion_ugv_map_gateway/map_gateway_node.hpp"

using namespace std::chrono_literals;
using husarion_ugv_map_gateway::MapGatewayNode;
namespace fs = std::filesystem;
namespace gw = husarion_ugv_map_gateway;

using Trigger = std_srvs::srv::Trigger;
using ManageNodes = nav2_msgs::srv::ManageLifecycleNodes;
using GetState = lifecycle_msgs::srv::GetState;
using SaveMap = nav2_msgs::srv::SaveMap;
using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

nav_msgs::msg::OccupancyGrid Grid(const std::string & frame, int marker, double origin_x = -1.0)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = frame;
  g.header.stamp.sec = 1000 + marker;
  g.info.resolution = 0.05f;
  g.info.width = 40;
  g.info.height = 20;
  g.info.origin.position.x = origin_x;
  g.info.origin.position.y = -0.5;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(800, 0);
  g.data[19 * 40 + 0] = 100;  // top-left of the picture
  g.data[marker % 800] = -1;
  return g;
}

class GatewayTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() override
  {
    static int n = 0;
    const std::string id = std::to_string(::getpid()) + "_" + std::to_string(++n);
    ns_ = "gwtest_" + id;
    dir_ = fs::temp_directory_path() / ("map_gateway_node_" + id);
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "maps");
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    helper_ = rclcpp::Node::make_shared("helper", "/" + ns_);
    exec_->add_node(helper_);
    state_sub_ = helper_->create_subscription<DiagnosticStatus>(
      "cockpit/nav/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](DiagnosticStatus::ConstSharedPtr m) {
        state_ = *m;
        state_times_.push_back(std::chrono::steady_clock::now());
      });
    tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(helper_);
    tf_timer_ = helper_->create_wall_timer(50ms, [this] { BroadcastTf(); });
  }

  void TearDown() override
  {
    gateway_.reset();
    map_node_.reset();
    exec_.reset();
    fs::remove_all(dir_);
  }

  std::string Frame(const std::string & name) const { return ns_ + "/" + name; }

  void BroadcastTf()
  {
    if (!tf_on_) {
      return;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = helper_->now();
    t.header.frame_id = Frame("map");
    t.child_frame_id = Frame("base_link");
    double x = 1.25, y = -0.5;
    if (path_) {
      std::tie(x, y) = path_(helper_->now().seconds());
    }
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = std::sin(0.3 / 2);
    t.transform.rotation.w = std::cos(0.3 / 2);
    tf_->sendTransform(t);
  }

  void StartGateway(std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("maps_directory", (dir_ / "maps").string()),
      rclcpp::Parameter("data_dir", (dir_ / "data").string()),
      rclcpp::Parameter("session_wait_s", 5.0),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    if (gateway_) {
      exec_->remove_node(gateway_);
      gateway_.reset();
    }
    gateway_ =
      std::make_shared<MapGatewayNode>(rclcpp::NodeOptions().parameter_overrides(params).arguments(
        {"--ros-args", "-r", "__ns:=/" + ns_}));
    exec_->add_node(gateway_);
  }

  void StopGateway()
  {
    exec_->remove_node(gateway_);
    gateway_.reset();
  }

  void StartMapPublisher(const nav_msgs::msg::OccupancyGrid & grid)
  {
    StopMapPublisher();
    static int n = 0;
    map_node_ = rclcpp::Node::make_shared("fake_map_server_" + std::to_string(++n), "/" + ns_);
    map_pub_ = map_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).reliable().transient_local());
    map_pub_->publish(grid);
    exec_->add_node(map_node_);
  }

  void StopMapPublisher()
  {
    if (map_node_) {
      exec_->remove_node(map_node_);
      map_pub_.reset();
      map_node_.reset();
    }
  }

  bool SpinUntil(const std::function<bool()> & done, std::chrono::duration<double> timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      exec_->spin_some(20ms);
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return done();
  }

  void SpinFor(std::chrono::duration<double> d)
  {
    SpinUntil([] { return false; }, d);
  }

  std::string Value(const std::string & key) const
  {
    if (!state_) {
      return "<no state>";
    }
    for (const auto & kv : state_->values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return "<missing>";
  }

  bool WaitValue(const std::string & key, const std::string & value, double timeout_s = 10.0)
  {
    const bool ok = SpinUntil(
      [&] { return Value(key) == value; }, std::chrono::duration<double>(timeout_s));
    if (!ok) {
      std::string all;
      if (state_) {
        for (const auto & kv : state_->values) {
          all += " " + kv.key + "=" + kv.value;
        }
      }
      ADD_FAILURE() << key << " is '" << Value(key) << "', waited for '" << value << "';" << all;
    }
    return ok;
  }

  template <typename SrvT>
  typename SrvT::Response::SharedPtr Call(
    const std::string & name, typename SrvT::Request::SharedPtr req, double timeout_s = 15.0)
  {
    auto client = helper_->create_client<SrvT>(name);
    if (!SpinUntil([&] { return client->service_is_ready(); }, 5s)) {
      return nullptr;
    }
    auto future = client->async_send_request(req);
    if (!SpinUntil(
          [&] { return future.wait_for(0s) == std::future_status::ready; },
          std::chrono::duration<double>(timeout_s))) {
      return nullptr;
    }
    return future.get();
  }

  // Nav2's lifecycle manager and the collision monitor, as far as the gateway sees them.
  void StartFakeLifecycle()
  {
    is_active_srv_ = helper_->create_service<Trigger>(
      "lifecycle_manager_navigation/is_active",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = nav_active_;
      });
    manage_srv_ = helper_->create_service<ManageNodes>(
      "lifecycle_manager_navigation/manage_nodes",
      [this](
        const std::shared_ptr<ManageNodes::Request> req,
        std::shared_ptr<ManageNodes::Response> res) {
        commands_.push_back(req->command);
        res->success = manage_success_;
      });
    cm_srv_ = helper_->create_service<GetState>(
      "collision_monitor/get_state",
      [this](const std::shared_ptr<GetState::Request>, std::shared_ptr<GetState::Response> res) {
        res->current_state.id = cm_state_;
      });
  }

  std::string ns_;
  fs::path dir_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Node::SharedPtr map_node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  std::shared_ptr<MapGatewayNode> gateway_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr state_sub_;
  std::optional<DiagnosticStatus> state_;
  std::vector<std::chrono::steady_clock::time_point> state_times_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  bool tf_on_ = true;
  std::function<std::pair<double, double>(double)> path_;
  rclcpp::Service<Trigger>::SharedPtr is_active_srv_;
  rclcpp::Service<ManageNodes>::SharedPtr manage_srv_;
  rclcpp::Service<GetState>::SharedPtr cm_srv_;
  bool nav_active_ = true;
  bool manage_success_ = true;
  uint8_t cm_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  std::vector<uint8_t> commands_;
};

}  // namespace

TEST_F(GatewayTest, LifecycleStates)
{
  StartGateway();
  ASSERT_TRUE(WaitValue("nav2", "absent"));
  EXPECT_EQ(Value("collision_monitor"), "absent");
  EXPECT_EQ(state_->name, "cockpit/nav");
  EXPECT_EQ(Value("schema"), "1");
  EXPECT_EQ(Value("frame"), Frame("map"));
  EXPECT_EQ(Value("mode"), "mapping");
  EXPECT_EQ(state_->hardware_id, Value("epoch"));
  EXPECT_EQ(state_->message, "not_aligned");
  EXPECT_EQ(state_->level, DiagnosticStatus::WARN);
  StartFakeLifecycle();
  ASSERT_TRUE(WaitValue("nav2", "active"));
  ASSERT_TRUE(WaitValue("collision_monitor", "active"));
  cm_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  ASSERT_TRUE(WaitValue("collision_monitor", "inactive"));
  nav_active_ = false;
  ASSERT_TRUE(WaitValue("nav2", "inactive"));
  cm_srv_.reset();
  ASSERT_TRUE(WaitValue("collision_monitor", "absent"));
}

TEST_F(GatewayTest, PauseAndResumeSendOnlyPauseAndResume)
{
  StartGateway();
  StartFakeLifecycle();
  auto res = Call<Trigger>("cockpit/nav/pause", std::make_shared<Trigger::Request>());
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->success);
  res = Call<Trigger>("cockpit/nav/resume", std::make_shared<Trigger::Request>());
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->success);
  manage_success_ = false;  // Nav2 says no: the trigger says no
  res = Call<Trigger>("cockpit/nav/pause", std::make_shared<Trigger::Request>());
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(
    commands_,
    (std::vector<uint8_t>{
      ManageNodes::Request::PAUSE, ManageNodes::Request::RESUME, ManageNodes::Request::PAUSE}));
  EXPECT_EQ(ManageNodes::Request::PAUSE, 1);
  EXPECT_EQ(ManageNodes::Request::RESUME, 2);
  manage_srv_.reset();  // no lifecycle manager: the trigger answers false at once
  res = Call<Trigger>("cockpit/nav/resume", std::make_shared<Trigger::Request>());
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(commands_.size(), 3u);
}

TEST_F(GatewayTest, EpochFollowsThePublisherAndSurvivesAGatewayRestart)
{
  StartMapPublisher(Grid(Frame("map"), 1));
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return Value("epoch").size() == 16; }, 10s));
  const std::string first = Value("epoch");
  SpinFor(2.5s);  // the publisher is bound
  // The grid grows and changes: no new epoch.
  map_pub_->publish(Grid(Frame("map"), 2, -3.0));
  SpinFor(2.5s);
  EXPECT_EQ(Value("epoch"), first);
  // slam_toolbox restarts: another GID, another epoch.
  StopMapPublisher();
  SpinFor(1.5s);
  EXPECT_EQ(Value("epoch"), first) << "a gap is no change";
  StartMapPublisher(Grid(Frame("map"), 1));
  ASSERT_TRUE(SpinUntil([&] { return Value("epoch") != first; }, 10s));
  const std::string second = Value("epoch");
  SpinFor(2.5s);  // the session file has the new GID
  // The gateway restarts alone: the same GID, so the same epoch.
  StopGateway();
  state_.reset();
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return state_.has_value(); }, 15s));
  EXPECT_EQ(Value("epoch"), second);
  // And restarted while the publisher is a new one: a new epoch.
  StopGateway();
  StartMapPublisher(Grid(Frame("map"), 1));
  SpinFor(1s);
  state_.reset();
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return state_.has_value(); }, 15s));
  EXPECT_NE(Value("epoch"), second);
  EXPECT_NE(Value("epoch"), first);
}

TEST_F(GatewayTest, SavedPairServesFromLLAndKeepsItsEpochAcrossAMapServerRestart)
{
  // A saved pair with its sidecar, aligned.
  const fs::path pair = dir_ / "maps" / "hall";
  fs::create_directories(pair);
  std::ofstream(pair / "map.png", std::ios::binary) << "png bytes";
  std::ofstream(pair / "map.yaml") << "image: map.png\nresolution: 0.05\n";
  gw::Sidecar sidecar;
  sidecar.epoch = "savedhall";
  sidecar.map_sha256 = gw::Sha256Hex("png bytes");
  gw::Alignment a;
  a.geo = {51.1, 17.03, 0.4};
  a.rms_m = 0.8;
  a.fix = gw::FixClass::kPlain;
  a.pairs = 40;
  sidecar.alignment = a;
  sidecar.saved_pose = gw::SavedPose{1.0, 2.0, 0.5};
  ASSERT_TRUE(gw::WriteSidecar(gw::SidecarPath((pair / "map.yaml").string()), sidecar));

  StartMapPublisher(Grid(Frame("map"), 5));
  StartGateway({rclcpp::Parameter("map_file_name", (pair / "map.yaml").string())});
  ASSERT_TRUE(WaitValue("epoch", "savedhall"));
  EXPECT_EQ(Value("align"), "held");
  EXPECT_EQ(Value("source"), "sidecar");
  EXPECT_EQ(Value("mode"), "amcl");
  EXPECT_EQ(Value("align_version"), "1");
  EXPECT_EQ(state_->level, DiagnosticStatus::OK);

  auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
  const gw::LatLon ll = gw::MapToLatLon(a.geo, 12.0, -7.0);
  req->ll_point.latitude = ll.lat;
  req->ll_point.longitude = ll.lon;
  auto res = Call<robot_localization::srv::FromLL>("base/fromLL", req);
  ASSERT_TRUE(res);
  EXPECT_NEAR(res->map_point.x, 12.0, 1e-6);
  EXPECT_NEAR(res->map_point.y, -7.0, 1e-6);
  auto back_req = std::make_shared<robot_localization::srv::ToLL::Request>();
  back_req->map_point.x = 12.0;
  back_req->map_point.y = -7.0;
  auto back = Call<robot_localization::srv::ToLL>("base/toLL", back_req);
  ASSERT_TRUE(back);
  EXPECT_NEAR(back->ll_point.latitude, ll.lat, 1e-9);
  EXPECT_NEAR(back->ll_point.longitude, ll.lon, 1e-9);
  // Nonsense in, NaN out (the agent rejects a non-finite answer).
  req->ll_point.latitude = 95.0;
  res = Call<robot_localization::srv::FromLL>("base/fromLL", req);
  ASSERT_TRUE(res);
  EXPECT_TRUE(std::isnan(res->map_point.x));

  // map_server restarts on the same map: same epoch.
  SpinFor(2.5s);
  StopMapPublisher();
  SpinFor(1.5s);
  StartMapPublisher(Grid(Frame("map"), 5));
  SpinFor(13s);  // past same_map_wait_s (10 s): the latched grid decided it, not the wait
  EXPECT_EQ(Value("epoch"), "savedhall");
  // A publisher with another map: a new epoch, no alignment, fromLL withdrawn.
  StopMapPublisher();
  SpinFor(1.5s);
  StartMapPublisher(Grid(Frame("map"), 6));
  ASSERT_TRUE(SpinUntil([&] { return Value("epoch") != "savedhall"; }, 10s));
  EXPECT_EQ(Value("align"), "not_aligned");
  EXPECT_EQ(Value("align_version"), "2");
  auto client = helper_->create_client<robot_localization::srv::FromLL>("base/fromLL");
  EXPECT_TRUE(SpinUntil([&] { return !client->service_is_ready(); }, 5s));
}

TEST_F(GatewayTest, FromLLIsNotServedWithoutAnAlignment)
{
  StartGateway();
  ASSERT_TRUE(WaitValue("align", "not_aligned"));
  auto client = helper_->create_client<robot_localization::srv::FromLL>("base/fromLL");
  auto toll = helper_->create_client<robot_localization::srv::ToLL>("base/toLL");
  auto pause = helper_->create_client<Trigger>("cockpit/nav/pause");
  ASSERT_TRUE(SpinUntil([&] { return pause->service_is_ready(); }, 5s));
  SpinFor(1.5s);
  EXPECT_FALSE(client->service_is_ready());
  EXPECT_FALSE(toll->service_is_ready());
}

TEST_F(GatewayTest, SaveWritesThePairAndConfinesTheName)
{
  // A fake map_saver that writes what Nav2's does: <url>.yaml naming <url>.png.
  int saves = 0;
  auto saver = helper_->create_service<SaveMap>(
    "map_saver/save_map",
    [&](const std::shared_ptr<SaveMap::Request> req, std::shared_ptr<SaveMap::Response> res) {
      ++saves;
      EXPECT_EQ(req->map_topic, "/" + ns_ + "/map");
      EXPECT_EQ(req->image_format, "png");
      std::ofstream(req->map_url + ".png", std::ios::binary) << "image " << saves;
      std::ofstream(req->map_url + ".yaml")
        << "image: " << fs::path(req->map_url).filename().string() << ".png\n";
      res->result = true;
    });
  std::string graph_file;
  auto slam = helper_->create_service<SerializePoseGraph>(
    "slam_toolbox/serialize_map", [&](
                                    const std::shared_ptr<SerializePoseGraph::Request> req,
                                    std::shared_ptr<SerializePoseGraph::Response> res) {
      graph_file = req->filename;
      res->result = SerializePoseGraph::Response::RESULT_SUCCESS;
    });
  StartMapPublisher(Grid(Frame("map"), 1));
  StartGateway();
  ASSERT_TRUE(WaitValue("pose", "ok"));
  const std::string service = "map_gateway/serialize_map";
  const std::vector<std::string> bad_names = {"../escape",          "a/b",  "",   "..", ".",
                                              std::string(65, 'x'), "/abs", "a b"};
  for (const std::string & bad : bad_names) {
    auto req = std::make_shared<SerializePoseGraph::Request>();
    req->filename = bad;
    auto res = Call<SerializePoseGraph>(service, req);
    ASSERT_TRUE(res) << bad;
    EXPECT_EQ(res->result, SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE) << bad;
  }
  EXPECT_EQ(saves, 0);
  EXPECT_TRUE(fs::is_empty(dir_ / "maps"));
  EXPECT_FALSE(fs::exists(dir_ / "escape"));

  auto req = std::make_shared<SerializePoseGraph::Request>();
  req->filename = "hall_1";
  auto res = Call<SerializePoseGraph>(service, req);
  ASSERT_TRUE(res);
  EXPECT_EQ(res->result, SerializePoseGraph::Response::RESULT_SUCCESS);
  const fs::path yaml = dir_ / "maps" / "hall_1" / "map.yaml";
  ASSERT_TRUE(fs::exists(yaml));
  ASSERT_TRUE(fs::exists(dir_ / "maps" / "hall_1" / "map.png"));
  EXPECT_EQ(graph_file, (dir_ / "maps" / "hall_1" / "map").string());
  const auto sidecar = gw::LoadMatchingSidecar(yaml.string());
  ASSERT_TRUE(sidecar.has_value());
  EXPECT_EQ(sidecar->epoch, Value("epoch"));
  EXPECT_FALSE(sidecar->alignment.has_value());  // saved before any fit
  ASSERT_TRUE(sidecar->saved_pose.has_value());
  EXPECT_NEAR(sidecar->saved_pose->x, 1.25, 1e-6);
  EXPECT_NEAR(sidecar->saved_pose->y, -0.5, 1e-6);
  EXPECT_NEAR(sidecar->saved_pose->yaw, 0.3, 1e-6);

  // Never over an existing pair.
  res = Call<SerializePoseGraph>(service, req);
  ASSERT_TRUE(res);
  EXPECT_EQ(res->result, SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE);
  EXPECT_EQ(saves, 1);

  // No map_saver (localization mode has none): refused, nothing written.
  saver.reset();
  SpinFor(0.5s);
  req->filename = "hall_2";
  res = Call<SerializePoseGraph>(service, req);
  ASSERT_TRUE(res);
  EXPECT_EQ(res->result, SerializePoseGraph::Response::RESULT_FAILED_TO_WRITE_FILE);
  EXPECT_FALSE(fs::exists(dir_ / "maps" / "hall_2" / "map.yaml"));
}

TEST_F(GatewayTest, MapPoseAndPlanProducts)
{
  std::vector<sensor_msgs::msg::CompressedImage> pngs;
  std::vector<nav_msgs::msg::MapMetaData> infos;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<nav_msgs::msg::Path> plans;
  auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto s1 = helper_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "cockpit/nav/map/png", latched,
    [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) { pngs.push_back(*m); });
  auto s2 = helper_->create_subscription<nav_msgs::msg::MapMetaData>(
    "cockpit/nav/map/info", latched,
    [&](nav_msgs::msg::MapMetaData::ConstSharedPtr m) { infos.push_back(*m); });
  auto s3 = helper_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "cockpit/nav/pose", rclcpp::SensorDataQoS(),
    [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { poses.push_back(*m); });
  auto s4 = helper_->create_subscription<nav_msgs::msg::Path>(
    "cockpit/nav/plan", rclcpp::QoS(1).reliable(),
    [&](nav_msgs::msg::Path::ConstSharedPtr m) { plans.push_back(*m); });
  auto plan_pub = helper_->create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).reliable());
  auto status_pub = helper_->create_publisher<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local());
  auto fix_pub = helper_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::SensorDataQoS());

  StartMapPublisher(Grid(Frame("map"), 1));
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return !pngs.empty() && !infos.empty(); }, 10s));
  EXPECT_EQ(pngs[0].format, "png");
  EXPECT_EQ(pngs[0].header.frame_id, Frame("map"));
  EXPECT_EQ(pngs[0].header.stamp, infos[0].map_load_time);  // the pairing rule
  EXPECT_EQ(infos[0].width, 40u);
  EXPECT_EQ(infos[0].height, 20u);
  EXPECT_FLOAT_EQ(infos[0].resolution, 0.05f);
  EXPECT_DOUBLE_EQ(infos[0].origin.position.x, -1.0);
  const auto image = gw::DecodeGreyPng(std::string(pngs[0].data.begin(), pngs[0].data.end()));
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->pixels[0], 0);    // the occupied cell at the largest y is row 0
  EXPECT_EQ(image->pixels[1], 254);  // free
  ASSERT_TRUE(WaitValue("map_stamp", "1001.000000000"));

  // The same grid again is not sent again.
  map_pub_->publish(Grid(Frame("map"), 1));
  SpinFor(1s);
  EXPECT_EQ(pngs.size(), 1u);

  // Pose at about 8 Hz from TF.
  poses.clear();
  SpinFor(2s);
  EXPECT_GE(poses.size(), 12u);
  EXPECT_LE(poses.size(), 18u);
  ASSERT_FALSE(poses.empty());
  EXPECT_EQ(poses.back().header.frame_id, Frame("map"));
  EXPECT_NEAR(poses.back().pose.position.x, 1.25, 1e-9);
  EXPECT_EQ(Value("pose"), "ok");
  tf_on_ = false;
  ASSERT_TRUE(WaitValue("pose", "none", 5.0));
  tf_on_ = true;

  // A dense plan: at most 200 poses, per-pose headers empty, the ends kept.
  action_msgs::msg::GoalStatusArray status;
  status.status_list.resize(1);
  status.status_list[0].status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  status_pub->publish(status);
  nav_msgs::msg::Path path;
  path.header.frame_id = Frame("map");
  for (int i = 0; i < 1500; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = Frame("map");
    p.pose.position.x = i * 0.02;
    p.pose.position.y = (i % 2) * 0.2;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  plan_pub->publish(path);
  ASSERT_TRUE(SpinUntil([&] { return !plans.empty(); }, 5s));
  EXPECT_LE(plans[0].poses.size(), 200u);
  EXPECT_GE(plans[0].poses.size(), 2u);
  EXPECT_DOUBLE_EQ(plans[0].poses.back().pose.position.x, 1499 * 0.02);
  for (const auto & p : plans[0].poses) {
    EXPECT_TRUE(p.header.frame_id.empty());
    EXPECT_EQ(p.header.stamp.sec, 0);
  }
  // A plan with a NaN is not published.
  path.poses[3].pose.position.x = std::nan("");
  plan_pub->publish(path);
  SpinFor(2s);
  EXPECT_EQ(plans.size(), 1u);
  // The goal ends: one empty path, sent no sooner than 1.25 s after the last plan.
  status.status_list[0].status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  status_pub->publish(status);
  ASSERT_TRUE(SpinUntil([&] { return plans.size() == 2; }, 5s));
  EXPECT_TRUE(plans[1].poses.empty());

  // clock_offset_s: fixes stamped 2.5 s behind this clock.
  EXPECT_EQ(Value("clock_offset_s"), "unknown");
  EXPECT_EQ(Value("fix_age_s"), "unknown");
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = rclcpp::Time(helper_->now().nanoseconds() - 2500000000LL);
    fix.status.status = 0;
    fix.latitude = 51.1;
    fix.longitude = 17.03;
    fix_pub->publish(fix);
    SpinFor(0.3s);
  }
  ASSERT_TRUE(SpinUntil(
    [&] {
      const std::string v = Value("clock_offset_s");
      return v != "unknown" && std::abs(std::stod(v) - 2.5) < 0.1;
    },
    5s))
    << Value("clock_offset_s");
  EXPECT_EQ(Value("fix"), "plain");
  EXPECT_NE(Value("fix_age_s"), "unknown");
  // Every value parses; the lane caps hold.
  for (const auto & kv : state_->values) {
    EXPECT_FALSE(kv.value.empty()) << kv.key;
  }
  EXPECT_LT(
    gw::DiagnosticStatusCdrSize(
      state_->name, state_->message, state_->hardware_id,
      [&] {
        std::vector<std::pair<std::string, std::string>> v;
        for (const auto & kv : state_->values) {
          v.emplace_back(kv.key, kv.value);
        }
        return v;
      }()),
    4096u);
}

TEST_F(GatewayTest, StateIsSpacedAndBeats)
{
  StartGateway();
  StartFakeLifecycle();
  ASSERT_TRUE(SpinUntil([&] { return state_.has_value(); }, 10s));
  state_times_.clear();
  // Changes every 0.2 s for 4 s: the state follows them no faster than every 0.6 s.
  for (int i = 0; i < 20; ++i) {
    cm_state_ = (i % 2) ? lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
                        : lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    SpinFor(0.2s);
  }
  SpinFor(3s);  // then only the heartbeat
  ASSERT_GE(state_times_.size(), 7u);
  for (size_t i = 1; i < state_times_.size(); ++i) {
    const double gap = std::chrono::duration<double>(state_times_[i] - state_times_[i - 1]).count();
    EXPECT_GE(gap, 0.5) << i;  // 0.6 s at the sender, less the delivery jitter
    EXPECT_LE(gap, 1.4) << i;  // the 1 s heartbeat
  }
}

namespace
{

// A saved pair with an aligned sidecar, as the gateway itself writes one.
std::string WritePair(
  const fs::path & dir, const std::string & epoch, const std::optional<gw::Alignment> & a)
{
  fs::create_directories(dir);
  std::ofstream(dir / "map.png", std::ios::binary) << "png of " << epoch;
  std::ofstream(dir / "map.yaml") << "image: map.png\nresolution: 0.05\n";
  gw::Sidecar sidecar;
  sidecar.epoch = epoch;
  sidecar.map_sha256 = gw::Sha256Hex("png of " + epoch);
  sidecar.alignment = a;
  sidecar.saved_pose = gw::SavedPose{0.5, 0.25, 0.1};
  const std::string yaml = (dir / "map.yaml").string();
  gw::WriteSidecar(gw::SidecarPath(yaml), sidecar);
  return yaml;
}

}  // namespace

TEST_F(GatewayTest, FixesAlignTheMapAndTheSidecarFollowsTheVersion)
{
  // The truth, and a sidecar whose alignment is 1 m off it.
  const gw::GeoRef truth{51.1, 17.03, 0.4};
  gw::Alignment off;
  const gw::LatLon shifted = gw::EnuToLatLon({truth.lat0, truth.lon0}, {0.0, 1.0});
  off.geo = {shifted.lat, shifted.lon, truth.yaw};
  off.rms_m = 0.05;
  off.fix = gw::FixClass::kRtkFixed;
  off.pairs = 30;
  const std::string yaml = WritePair(dir_ / "maps" / "field", "fieldepoch", off);

  // The robot drives a 12 m square at 4 m/s; an RTK receiver reports it 5 times a second.
  const double t0 = helper_->now().seconds();
  path_ = [t0](double t) -> std::pair<double, double> {
    const double s = std::fmod(4.0 * (t - t0), 48.0);
    if (s < 12) {
      return {s, 0.0};
    }
    if (s < 24) {
      return {12.0, s - 12};
    }
    if (s < 36) {
      return {36 - s, 12.0};
    }
    return {0.0, 48 - s};
  };
  auto fix_pub = helper_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::SensorDataQoS());
  auto fix_timer = helper_->create_wall_timer(200ms, [&] {
    const rclcpp::Time stamp(helper_->now().nanoseconds() - 100000000LL);
    const auto [x, y] = path_(stamp.seconds());
    const gw::LatLon ll = gw::MapToLatLon(truth, x, y);
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.status.status = 2;  // GBAS: RTK
    fix.latitude = ll.lat;
    fix.longitude = ll.lon;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix.position_covariance[0] = fix.position_covariance[4] = 0.0004;
    fix_pub->publish(fix);
  });

  StartMapPublisher(Grid(Frame("map"), 7));
  StartGateway({
    rclcpp::Parameter("map_file_name", yaml),
    rclcpp::Parameter("align.better_s", 1.0),
    rclcpp::Parameter("align.worse_s", 1.0),
    rclcpp::Parameter("align.disagree_s", 2.0),
  });
  ASSERT_TRUE(WaitValue("epoch", "fieldepoch"));
  // Held from the sidecar, then the fixes disagree with it by 1 m (over 3 x 0.2 m): poor, and a
  // fit of their own replaces it once the track is 10 m wide.
  ASSERT_TRUE(WaitValue("align", "aligned", 40.0));
  EXPECT_EQ(Value("source"), "fit");
  EXPECT_EQ(Value("fix"), "rtk_fixed");
  EXPECT_EQ(Value("epoch"), "fieldepoch");
  EXPECT_GE(std::stoi(Value("align_version")), 2);
  EXPECT_NEAR(std::stod(Value("lat0")), truth.lat0, 2e-7);  // about 2 cm
  EXPECT_NEAR(std::stod(Value("lon0")), truth.lon0, 3e-7);
  EXPECT_NEAR(std::stod(Value("yaw")), truth.yaw, 0.01);
  EXPECT_EQ(Value("err_m"), "0.200");

  // The pair it runs on now carries the new alignment, with its hash, epoch and pose kept.
  ASSERT_TRUE(SpinUntil(
    [&] {
      const auto s = gw::ReadSidecar(gw::SidecarPath(yaml));
      return s && s->alignment && std::abs(s->alignment->geo.lat0 - truth.lat0) < 2e-7;
    },
    5s));
  const auto sidecar = gw::LoadMatchingSidecar(yaml);
  ASSERT_TRUE(sidecar.has_value());
  EXPECT_EQ(sidecar->epoch, "fieldepoch");
  ASSERT_TRUE(sidecar->saved_pose.has_value());
  EXPECT_DOUBLE_EQ(sidecar->saved_pose->x, 0.5);

  // fromLL is served and agrees with the truth near the track.
  auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
  const gw::LatLon ll = gw::MapToLatLon(truth, 6.0, 6.0);
  req->ll_point.latitude = ll.lat;
  req->ll_point.longitude = ll.lon;
  auto res = Call<robot_localization::srv::FromLL>("base/fromLL", req);
  ASSERT_TRUE(res);
  EXPECT_NEAR(res->map_point.x, 6.0, 0.05);
  EXPECT_NEAR(res->map_point.y, 6.0, 0.05);

  // The fixes stop: held after 10 s, fromLL still served.
  fix_timer->cancel();
  ASSERT_TRUE(WaitValue("align", "held", 15.0));
  EXPECT_EQ(state_->level, DiagnosticStatus::OK);
}

TEST_F(GatewayTest, AnotherMapFileNameIsAnotherEpoch)
{
  const std::string a = WritePair(dir_ / "maps" / "a", "epocha", std::nullopt);
  const std::string b = WritePair(dir_ / "maps" / "b", "epochb", std::nullopt);
  StartMapPublisher(Grid(Frame("map"), 1));
  StartGateway({rclcpp::Parameter("map_file_name", a)});
  ASSERT_TRUE(WaitValue("epoch", "epocha"));
  EXPECT_EQ(Value("align"), "not_aligned");  // saved before any fit
  gateway_->set_parameter(rclcpp::Parameter("map_file_name", b));
  ASSERT_TRUE(WaitValue("epoch", "epochb"));
  gateway_->set_parameter(rclcpp::Parameter("map_file_name", ""));
  ASSERT_TRUE(SpinUntil([&] { return Value("epoch").size() == 16; }, 10s));
  EXPECT_EQ(Value("mode"), "mapping");
}

TEST_F(GatewayTest, ALatchedMapIsSentAfterTheGatewayRestartsOnItsSession)
{
  // map_server publishes once. A gateway that restarts on its session file still sends it.
  StartMapPublisher(Grid(Frame("map"), 3));
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return Value("epoch").size() == 16; }, 10s));
  const std::string epoch = Value("epoch");
  SpinFor(2.5s);  // the session file holds the publisher's GID
  StopGateway();
  state_.reset();
  int pngs = 0;
  auto sub = helper_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "cockpit/nav/map/png", rclcpp::QoS(1).reliable().transient_local(),
    [&](sensor_msgs::msg::CompressedImage::ConstSharedPtr) { ++pngs; });
  StartGateway();
  ASSERT_TRUE(SpinUntil([&] { return state_.has_value() && pngs > 0; }, 15s))
    << "state " << state_.has_value() << ", pngs " << pngs;
  EXPECT_EQ(Value("epoch"), epoch);
  EXPECT_EQ(Value("map_stamp"), "1003.000000000");
}
