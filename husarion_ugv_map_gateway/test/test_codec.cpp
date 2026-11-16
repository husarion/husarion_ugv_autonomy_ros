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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "husarion_ugv_map_gateway/codec.hpp"

using husarion_ugv_map_gateway::DecodeGreyPng;
using husarion_ugv_map_gateway::EncodeGreyPng;
using husarion_ugv_map_gateway::Sha256Hex;

TEST(Sha256, KnownVectors)
{
  EXPECT_EQ(Sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(Sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(
    Sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
    "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(
    Sha256Hex(std::string(1000000, 'a')),
    "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, EveryPaddingLength)
{
  // The lengths around the one- and two-block padding boundaries, against Python's hashlib.
  const std::vector<std::pair<size_t, std::string>> cases = {
    {55, "d5e285683cd4efc02d021a5c62014694958901005d6f71e89e0989fac77e4072"},
    {56, "04c26261370ee7541549d16dee320c723e3fd14671e66a099afe0a377c16888e"},
    {63, "75220b47218278e656f2013bb8f0c455a25eaf01e86c64924e9d48d89776d6f2"},
    {64, "7ce100971f64e7001e8fe5a51973ecdfe1ced42befe7ee8d5fd6219506b5393c"},
    {119, "000b48d4edf0fa7bee3c6236ecd2785baa5db4eeb8bb54341b029e0d9fa5fb0c"},
    {120, "13f05a0b594787f5ecd315edc96141bd3243203d1b7d4f0836f37308b276ba98"},
  };
  for (const auto & [n, digest] : cases) {
    EXPECT_EQ(Sha256Hex(std::string(n, 'x')), digest) << n;
  }
}

TEST(Png, RoundTrip)
{
  const uint32_t w = 37, h = 11;
  std::vector<uint8_t> pixels(w * h);
  for (size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = static_cast<uint8_t>((i * 7) % 256);
  }
  const std::string png = EncodeGreyPng(pixels, w, h);
  ASSERT_EQ(png.substr(1, 3), "PNG");
  const auto image = DecodeGreyPng(png);
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->width, w);
  EXPECT_EQ(image->height, h);
  EXPECT_EQ(image->pixels, pixels);
}

TEST(Png, CorruptIsRejected)
{
  std::string png = EncodeGreyPng(std::vector<uint8_t>(16, 254), 4, 4);
  png[png.size() / 2] ^= 0x55;
  EXPECT_FALSE(DecodeGreyPng(png).has_value());
  EXPECT_FALSE(DecodeGreyPng("not a png").has_value());
}
