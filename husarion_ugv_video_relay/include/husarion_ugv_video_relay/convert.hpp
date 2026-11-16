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

#ifndef HUSARION_UGV_VIDEO_RELAY_CONVERT_HPP_
#define HUSARION_UGV_VIDEO_RELAY_CONVERT_HPP_

#include <optional>
#include <string>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>

namespace husarion_ugv_video_relay
{

// True when an FFMPEGPacket's `encoding` names an H.264 bitstream. Its first ';'-separated token
// is the libav encoder's name, not the codec: `libx264` (depthai's label for the OAK's on-chip
// encoder), `h264_nvenc`, `h264_vaapi`, `h264_v4l2m2m`, or plain `h264`. Empty, `hevc_*`, `libx265`
// and the rest are not H.264. The same rule as the Cockpit camera node's `is_h264_codec`.
bool IsH264Encoding(const std::string & encoding);

// The packet as a CompressedVideo with format `h264`: the bytes moved unchanged, the stamp and
// frame_id carried (frame_id replaced when `frame_id` is not empty). `std::nullopt` for a packet
// that is not H.264, which the relay drops rather than mislabel.
std::optional<foxglove_msgs::msg::CompressedVideo> ToCompressedVideo(
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket && packet, const std::string & frame_id = "");

}  // namespace husarion_ugv_video_relay

#endif  // HUSARION_UGV_VIDEO_RELAY_CONVERT_HPP_
