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

#include "husarion_ugv_video_relay/relay_node.hpp"

#include <memory>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

#include "husarion_ugv_video_relay/convert.hpp"

namespace husarion_ugv_video_relay
{

FfmpegToCompressedVideo::FfmpegToCompressedVideo(const rclcpp::NodeOptions & options)
: rclcpp::Node("ffmpeg_to_compressed_video", options)
{
  frame_id_ = declare_parameter<std::string>("frame_id", "");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(
    "output", rclcpp::QoS(10), pub_options);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  sub_ = create_subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
    "input", rclcpp::SensorDataQoS(),
    [this](ffmpeg_image_transport_msgs::msg::FFMPEGPacket::UniquePtr packet) {
      OnPacket(std::move(packet));
    },
    sub_options);

  RCLCPP_INFO(
    get_logger(), "Relaying H.264 from %s (FFMPEGPacket) to %s (CompressedVideo)",
    sub_->get_topic_name(), pub_->get_topic_name());
}

void FfmpegToCompressedVideo::OnPacket(
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket::UniquePtr packet)
{
  const std::string encoding = packet->encoding;
  auto video = ToCompressedVideo(std::move(*packet), frame_id_);
  if (!video) {
    ++dropped_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Dropping packets with encoding '%s': only H.264 is relayed (%lu dropped so far)",
      encoding.c_str(), static_cast<unsigned long>(dropped_));
    return;
  }
  ++relayed_;
  pub_->publish(std::make_unique<foxglove_msgs::msg::CompressedVideo>(std::move(*video)));
}

}  // namespace husarion_ugv_video_relay

RCLCPP_COMPONENTS_REGISTER_NODE(husarion_ugv_video_relay::FfmpegToCompressedVideo)
