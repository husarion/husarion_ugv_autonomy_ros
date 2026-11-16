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

// The packet-to-CompressedVideo rule with no ROS graph: which encodings are H.264, and that the
// bytes, stamp and frame_id cross unchanged.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "husarion_ugv_video_relay/convert.hpp"

using husarion_ugv_video_relay::IsH264Encoding;
using husarion_ugv_video_relay::ToCompressedVideo;

namespace
{

ffmpeg_image_transport_msgs::msg::FFMPEGPacket Packet(const std::string & encoding)
{
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket p;
  p.header.stamp.sec = 1794868976;
  p.header.stamp.nanosec = 566849134;
  p.header.frame_id = "oak_rgb_camera_optical_frame";
  p.width = 1280;
  p.height = 720;
  p.encoding = encoding;
  p.flags = 1;
  // An access unit as the OAK sends it: AUD, SPS, PPS, IDR slice.
  p.data = {0, 0, 0, 1,    0x09, 0x10, 0,    0, 0, 1, 0x67, 0x42, 0xc0, 0x1f, 0,
            0, 0, 1, 0x68, 0xce, 0x3c, 0x80, 0, 0, 0, 1,    0x65, 0x88, 0x84};
  return p;
}

}  // namespace

TEST(Convert, H264FamilyIsAccepted)
{
  for (const char * e :
       {"h264", "libx264", "h264_nvenc", "h264_vaapi", "h264_v4l2m2m",
        "libx264;preset=ultrafast"}) {
    EXPECT_TRUE(IsH264Encoding(e)) << e;
  }
}

TEST(Convert, OtherCodecsAreRefused)
{
  for (const char * e : {"", "hevc", "hevc_nvenc", "libx265", "av1", "vp9", "mjpeg", "x264"}) {
    EXPECT_FALSE(IsH264Encoding(e)) << e;
  }
}

TEST(Convert, BytesStampAndFrameCross)
{
  const auto in = Packet("libx264");
  auto out = ToCompressedVideo(Packet("libx264"));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->format, "h264");
  EXPECT_EQ(out->data, in.data);
  EXPECT_EQ(out->timestamp.sec, in.header.stamp.sec);
  EXPECT_EQ(out->timestamp.nanosec, in.header.stamp.nanosec);
  EXPECT_EQ(out->frame_id, "oak_rgb_camera_optical_frame");
}

TEST(Convert, FrameIdOverride)
{
  auto out = ToCompressedVideo(Packet("libx264"), "panther_0957/oak_rgb_camera_optical_frame");
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->frame_id, "panther_0957/oak_rgb_camera_optical_frame");
}

TEST(Convert, HevcIsNotLabelledH264)
{
  EXPECT_FALSE(ToCompressedVideo(Packet("hevc_nvenc")).has_value());
  EXPECT_FALSE(ToCompressedVideo(Packet("")).has_value());
}
