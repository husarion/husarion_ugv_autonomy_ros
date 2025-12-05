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

#include <chrono>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "husarion_ugv_docking/dock_database_updater_node.hpp"

static constexpr char kFilepath[] = "/tmp/dock_database.yaml";
static constexpr char kDefaultFrame[] = "map";
static constexpr char kDefaultDockName[] = "main";
static constexpr char kDefaultDockType[] = "charging_dock";

class DockDatabaseUpdaterWrapper : public husarion_ugv_docking::DockDatabaseUpdaterNode
{
public:
  DockDatabaseUpdaterWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : husarion_ugv_docking::DockDatabaseUpdaterNode(node_name, options)
  {
  }

  void PoseCallback(
    const std::string & dock_name, const std::string & dock_type,
    const husarion_ugv_docking::PoseStampedMsg::SharedPtr msg)
  {
    husarion_ugv_docking::DockDatabaseUpdaterNode::PoseCallback(dock_name, dock_type, msg);
  }

  bool UpdateDatabaseFile(
    const std::string & dock_name, const std::string & dock_type,
    const husarion_ugv_docking::PoseStampedMsg::SharedPtr pose)
  {
    return husarion_ugv_docking::DockDatabaseUpdaterNode::UpdateDatabaseFile(
      dock_name, dock_type, pose);
  }

  YAML::Node UpdateDockDatabase(
    const std::string & dock_name, const std::string & dock_type,
    const husarion_ugv_docking::PoseStampedMsg::SharedPtr pose)
  {
    return husarion_ugv_docking::DockDatabaseUpdaterNode::UpdateDockDatabase(
      dock_name, dock_type, pose);
  }

  husarion_ugv_docking::PoseStampedMsg::SharedPtr CreateInitialPose(
    const std::string & frame, const std::vector<double> & pose_vec)
  {
    return husarion_ugv_docking::DockDatabaseUpdaterNode::CreateInitialPose(frame, pose_vec);
  }
};

class TestDockDatabaseUpdater : public ::testing::Test
{
protected:
  TestDockDatabaseUpdater();
  void CreateDockDatabaseUpdaterNode(const std::vector<rclcpp::Parameter> & params);

  std::shared_ptr<DockDatabaseUpdaterWrapper> dock_database_updater_node_;
};

TestDockDatabaseUpdater::TestDockDatabaseUpdater()
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("dock_database_filepath", kFilepath);
  CreateDockDatabaseUpdaterNode(params);
}

void TestDockDatabaseUpdater::CreateDockDatabaseUpdaterNode(
  const std::vector<rclcpp::Parameter> & params)
{
  if (dock_database_updater_node_) {
    dock_database_updater_node_.reset();
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  dock_database_updater_node_ = std::make_shared<DockDatabaseUpdaterWrapper>(
    "dock_database_updater_node_for_test", options);
}

TEST_F(TestDockDatabaseUpdater, CreateInitialPoseCreatesValidPose)
{
  std::vector<double> pose_vec = {1.0, 2.0, M_PI / 2};

  auto pose = dock_database_updater_node_->CreateInitialPose(kDefaultFrame, pose_vec);

  EXPECT_EQ(pose->header.frame_id, kDefaultFrame);
  EXPECT_DOUBLE_EQ(pose->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose->pose.position.z, 0.0);

  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  EXPECT_DOUBLE_EQ(pose->pose.orientation.x, q.x());
  EXPECT_DOUBLE_EQ(pose->pose.orientation.y, q.y());
  EXPECT_DOUBLE_EQ(pose->pose.orientation.z, q.z());
  EXPECT_DOUBLE_EQ(pose->pose.orientation.w, q.w());
}

TEST_F(TestDockDatabaseUpdater, UpdateDockDatabaseCreatesValidYAMLWithDefaultParameters)
{
  YAML::Node yaml = YAML::LoadFile(kFilepath);

  EXPECT_TRUE(yaml.IsMap());
  ASSERT_EQ(yaml["docks"].size(), 1);
  EXPECT_NO_THROW(yaml["docks"][kDefaultDockName]);
  EXPECT_EQ(yaml["docks"][kDefaultDockName]["type"].as<std::string>(), kDefaultDockType);
  EXPECT_EQ(yaml["docks"][kDefaultDockName]["frame"].as<std::string>(), kDefaultFrame);

  auto pose_yaml = yaml["docks"][kDefaultDockName]["pose"];
  ASSERT_TRUE(pose_yaml.IsSequence());
  ASSERT_EQ(pose_yaml.size(), 3);
  EXPECT_DOUBLE_EQ(pose_yaml[0].as<double>(), 0.0);
  EXPECT_DOUBLE_EQ(pose_yaml[1].as<double>(), 0.0);
  EXPECT_DOUBLE_EQ(pose_yaml[2].as<double>(), 0.0);
}

TEST_F(TestDockDatabaseUpdater, UpdateDatabaseFileHandlesExceptions)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("dock_database_filepath", "/invalid_path/dock_database.yaml");
  EXPECT_THROW(CreateDockDatabaseUpdaterNode(params), std::runtime_error);
}

TEST_F(TestDockDatabaseUpdater, UpdateDatabaseFileHandlesSavingFile)
{
  std::string dock_name = "dock1";
  std::string dock_type = "typeA";
  std::vector<double> pose_vec = {1.0, 2.0, M_PI / 2};

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("docks", std::vector<std::string>({dock_name}));
  params.emplace_back(dock_name + ".type", dock_type);
  params.emplace_back(dock_name + ".pose", pose_vec);
  params.emplace_back(dock_name + ".frame", kDefaultFrame);

  auto pose = dock_database_updater_node_->CreateInitialPose(kDefaultFrame, pose_vec);
  bool result = dock_database_updater_node_->UpdateDatabaseFile(dock_name, dock_type, pose);
  EXPECT_TRUE(result);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return ret;
}
