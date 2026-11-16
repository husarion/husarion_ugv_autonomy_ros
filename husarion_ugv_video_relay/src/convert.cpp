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

#include "husarion_ugv_video_relay/convert.hpp"

#include <optional>
#include <string>
#include <utility>

namespace husarion_ugv_video_relay
{

bool IsH264Encoding(const std::string & encoding)
{
  const std::string token = encoding.substr(0, encoding.find(';'));
  return token == "h264" || token == "libx264" || token.rfind("h264_", 0) == 0;
}

std::optional<foxglove_msgs::msg::CompressedVideo> ToCompressedVideo(
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket && packet, const std::string & frame_id)
{
  if (!IsH264Encoding(packet.encoding)) {
    return std::nullopt;
  }
  foxglove_msgs::msg::CompressedVideo video;
  video.timestamp = packet.header.stamp;
  video.frame_id = frame_id.empty() ? std::move(packet.header.frame_id) : frame_id;
  video.data = std::move(packet.data);
  video.format = "h264";
  return video;
}

}  // namespace husarion_ugv_video_relay
