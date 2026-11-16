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

// The relay over the real middleware: a published FFMPEGPacket arrives as a CompressedVideo with
// its bytes, stamp and frame_id; a non-H.264 packet is dropped; the remapped topic names hold.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <rclcpp/rclcpp.hpp>

#include "husarion_ugv_video_relay/relay_node.hpp"

using namespace std::chrono_literals;

class RelayNode : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "-r", "__ns:=/panther_0957", "-r", "input:=oak/rgb/image_raw/compressed", "-r",
       "output:=oak/rgb/image_raw/h264"});
    relay_ = std::make_shared<husarion_ugv_video_relay::FfmpegToCompressedVideo>(options);
    peer_ = std::make_shared<rclcpp::Node>("relay_test_peer", "/panther_0957");
    pub_ = peer_->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
      "oak/rgb/image_raw/compressed", rclcpp::QoS(10));
    sub_ = peer_->create_subscription<foxglove_msgs::msg::CompressedVideo>(
      "oak/rgb/image_raw/h264", rclcpp::SensorDataQoS(),
      [this](foxglove_msgs::msg::CompressedVideo::SharedPtr m) { received_.push_back(*m); });
    exec_.add_node(relay_);
    exec_.add_node(peer_);
    ASSERT_TRUE(SpinUntil(
      [this] { return pub_->get_subscription_count() > 0 && sub_->get_publisher_count() > 0; }));
  }

  void TearDown() override
  {
    exec_.remove_node(relay_);
    exec_.remove_node(peer_);
  }

  template <typename Pred>
  bool SpinUntil(Pred pred, std::chrono::seconds timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(50ms);
      if (pred()) {
        return true;
      }
    }
    return false;
  }

  static ffmpeg_image_transport_msgs::msg::FFMPEGPacket Packet(const char * encoding, int n)
  {
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket p;
    p.header.stamp.sec = 100 + n;
    p.header.stamp.nanosec = 7;
    p.header.frame_id = "oak_rgb_camera_optical_frame";
    p.encoding = encoding;
    p.width = 1280;
    p.height = 720;
    p.data = std::vector<std::uint8_t>(16000, static_cast<std::uint8_t>(n));
    return p;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<husarion_ugv_video_relay::FfmpegToCompressedVideo> relay_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr pub_;
  rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr sub_;
  std::vector<foxglove_msgs::msg::CompressedVideo> received_;
};

TEST_F(RelayNode, TopicNamesFollowTheRemaps)
{
  const auto topics = peer_->get_topic_names_and_types();
  ASSERT_TRUE(topics.count("/panther_0957/oak/rgb/image_raw/h264"));
  EXPECT_EQ(
    topics.at("/panther_0957/oak/rgb/image_raw/h264").front(), "foxglove_msgs/msg/CompressedVideo");
}

TEST_F(RelayNode, H264PacketsArriveUnchanged)
{
  for (int n = 1; n <= 3; ++n) {
    pub_->publish(Packet("libx264", n));
    ASSERT_TRUE(SpinUntil([&] { return received_.size() == static_cast<size_t>(n); }));
  }
  for (int n = 1; n <= 3; ++n) {
    const auto & m = received_[n - 1];
    EXPECT_EQ(m.format, "h264");
    EXPECT_EQ(m.timestamp.sec, 100 + n);
    EXPECT_EQ(m.timestamp.nanosec, 7u);
    EXPECT_EQ(m.frame_id, "oak_rgb_camera_optical_frame");
    EXPECT_EQ(m.data, Packet("libx264", n).data);
  }
  EXPECT_EQ(relay_->relayed(), 3u);
}

TEST_F(RelayNode, NonH264IsDropped)
{
  pub_->publish(Packet("hevc_nvenc", 1));
  ASSERT_TRUE(SpinUntil([&] { return relay_->dropped() == 1u; }));
  pub_->publish(Packet("h264", 2));
  ASSERT_TRUE(SpinUntil([&] { return received_.size() == 1u; }));
  EXPECT_EQ(received_[0].timestamp.sec, 102);
  EXPECT_EQ(relay_->relayed(), 1u);
}
