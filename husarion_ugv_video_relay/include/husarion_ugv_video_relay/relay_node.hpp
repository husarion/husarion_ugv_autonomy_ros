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

#ifndef HUSARION_UGV_VIDEO_RELAY_RELAY_NODE_HPP_
#define HUSARION_UGV_VIDEO_RELAY_RELAY_NODE_HPP_

#include <cstdint>
#include <string>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <rclcpp/rclcpp.hpp>

namespace husarion_ugv_video_relay
{

// Copies each H.264 FFMPEGPacket on `input` into a foxglove_msgs/CompressedVideo on `output`, with
// no decode and no re-encode. It exists because the airlock image of Husarion UGV OS carries
// CompressedVideo but not FFMPEGPacket (SPEC-MAP-NAV D57, AIRLOCK-DEBT A13); it goes once the
// airlock carries FFMPEGPacket itself.
//
// Parameters: `frame_id` (string, default empty: carry the packet's own).
// Topics: `input` (subscribed, best effort) and `output` (published, reliable, depth 10, as
// depthai publishes); both remappable, and their QoS can be overridden with the standard
// `qos_overrides.<topic>.*` parameters.
class FfmpegToCompressedVideo : public rclcpp::Node
{
public:
  explicit FfmpegToCompressedVideo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::uint64_t relayed() const { return relayed_; }
  std::uint64_t dropped() const { return dropped_; }

private:
  void OnPacket(ffmpeg_image_transport_msgs::msg::FFMPEGPacket::UniquePtr packet);

  std::string frame_id_;
  std::uint64_t relayed_ = 0;
  std::uint64_t dropped_ = 0;
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr pub_;
  rclcpp::Subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr sub_;
};

}  // namespace husarion_ugv_video_relay

#endif  // HUSARION_UGV_VIDEO_RELAY_RELAY_NODE_HPP_
