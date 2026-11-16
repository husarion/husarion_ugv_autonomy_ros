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

// §11.2: the save name refuses a path separator or `..`; a sidecar whose map_sha256 does not
// match is ignored; the sidecar round trip (C++ side; the C++ to R1's Python reader round trip is
// husarion_ugv_navigation's test_sidecar_roundtrip); the session file.

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/files.hpp"

using namespace husarion_ugv_map_gateway;
namespace fs = std::filesystem;

namespace
{

class TempDir : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() /
           ("map_gateway_files_" + std::to_string(::getpid()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }

  std::string Write(const std::string & name, const std::string & content)
  {
    const std::string path = (dir_ / name).string();
    std::ofstream(path, std::ios::binary) << content;
    return path;
  }

  fs::path dir_;
};

Alignment SomeAlignment()
{
  Alignment a;
  a.geo = {51.107912345, 17.038512345, -1.234567891};
  a.rms_m = 1.25;
  a.extent_m = 42.5;
  a.pairs = 311;
  a.fix = FixClass::kRtkFloat;
  a.fitted_at = 1763300000.5;
  return a;
}

}  // namespace

TEST(SaveName, Confined)
{
  EXPECT_TRUE(ValidSaveName("hall"));
  EXPECT_TRUE(ValidSaveName("Hall_2-b"));
  EXPECT_TRUE(ValidSaveName(std::string(64, 'a')));
  EXPECT_FALSE(ValidSaveName(std::string(65, 'a')));
  EXPECT_FALSE(ValidSaveName(""));
  EXPECT_FALSE(ValidSaveName(".."));
  EXPECT_FALSE(ValidSaveName("."));
  EXPECT_FALSE(ValidSaveName("../etc"));
  EXPECT_FALSE(ValidSaveName("a/b"));
  EXPECT_FALSE(ValidSaveName("/abs"));
  EXPECT_FALSE(ValidSaveName("a\\b"));
  EXPECT_FALSE(ValidSaveName("a b"));
  EXPECT_FALSE(ValidSaveName("a.b"));
  EXPECT_FALSE(ValidSaveName("a\nb"));
  EXPECT_FALSE(ValidSaveName(std::string("a\0b", 3)));
  EXPECT_FALSE(ValidSaveName("zażółć"));
}

TEST(Epochs, NewAndValid)
{
  const std::string a = NewEpoch(), b = NewEpoch();
  EXPECT_EQ(a.size(), 16u);
  EXPECT_NE(a, b);
  EXPECT_TRUE(ValidEpoch(a));
  EXPECT_FALSE(ValidEpoch(""));
  EXPECT_FALSE(ValidEpoch("has space"));
}

TEST_F(TempDir, SidecarPathAndImage)
{
  EXPECT_EQ(SidecarPath("/maps/hall/map.yaml"), "/maps/hall/map.georef.yaml");
  const std::string yaml = Write("map.yaml", "image: map.png\nresolution: 0.05\n");
  EXPECT_EQ(*MapImagePath(yaml), (dir_ / "map.png").string());
  const std::string abs = Write("abs.yaml", "image: /elsewhere/x.pgm\n");
  EXPECT_EQ(*MapImagePath(abs), "/elsewhere/x.pgm");
  const std::string none = Write("none.yaml", "resolution: 0.05\n");
  EXPECT_FALSE(MapImagePath(none).has_value());
}

TEST_F(TempDir, SidecarRoundTrip)
{
  Sidecar s;
  s.epoch = "0123456789abcdef";
  s.map_sha256 = Sha256Hex("image");
  s.alignment = SomeAlignment();
  s.saved_pose = SavedPose{1.5, -2.25, 3.0};
  const std::string path = (dir_ / "map.georef.yaml").string();
  ASSERT_TRUE(WriteSidecar(path, s));
  const auto back = ReadSidecar(path);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->epoch, s.epoch);
  EXPECT_EQ(back->map_sha256, s.map_sha256);
  ASSERT_TRUE(back->alignment.has_value());
  EXPECT_NEAR(back->alignment->geo.lat0, 51.107912345, 1e-9);
  EXPECT_NEAR(back->alignment->geo.lon0, 17.038512345, 1e-9);
  EXPECT_NEAR(back->alignment->geo.yaw, -1.234567891, 1e-9);
  EXPECT_EQ(back->alignment->pairs, 311);
  EXPECT_EQ(back->alignment->fix, FixClass::kRtkFloat);
  EXPECT_EQ(back->alignment->source, Source::kSidecar);
  ASSERT_TRUE(back->saved_pose.has_value());
  EXPECT_DOUBLE_EQ(back->saved_pose->x, 1.5);
  EXPECT_DOUBLE_EQ(back->saved_pose->y, -2.25);
  EXPECT_DOUBLE_EQ(back->saved_pose->yaw, 3.0);
  // No temporary file is left beside it.
  int files = 0;
  for (const auto & entry : fs::directory_iterator(dir_)) {
    (void)entry;
    ++files;
  }
  EXPECT_EQ(files, 1);
}

TEST_F(TempDir, SidecarWithoutAlignment)
{
  // A save made before any fit: the epoch, the hash and saved_pose, no alignment (§5.5).
  Sidecar s;
  s.epoch = "abc";
  s.map_sha256 = "0000";
  s.saved_pose = SavedPose{0.0, 0.0, 0.0};
  const auto back = ParseSidecar(SidecarYaml(s));
  ASSERT_TRUE(back.has_value());
  EXPECT_FALSE(back->alignment.has_value());
  EXPECT_TRUE(back->saved_pose.has_value());
  EXPECT_EQ(SidecarYaml(s).find("lat0"), std::string::npos);
}

TEST_F(TempDir, EpochThatLooksLikeANumberStaysAString)
{
  Sidecar s;
  s.epoch = "1234e567";
  s.map_sha256 = "00112233";
  const auto back = ParseSidecar(SidecarYaml(s));
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->epoch, "1234e567");
  EXPECT_EQ(back->map_sha256, "00112233");
}

TEST_F(TempDir, BrokenSidecarsAreRejected)
{
  EXPECT_FALSE(ParseSidecar("schema: 2\nepoch: a\nmap_sha256: b\n").has_value());
  EXPECT_FALSE(ParseSidecar("schema: 1\nmap_sha256: b\n").has_value());
  EXPECT_FALSE(ParseSidecar("schema: 1\nepoch: a\n").has_value());
  EXPECT_FALSE(ParseSidecar("schema: 1\nepoch: a\nmap_sha256: b\nlat0: 1.0\n").has_value());
  EXPECT_FALSE(ParseSidecar(": : :").has_value());
  EXPECT_FALSE(ReadSidecar((dir_ / "missing.yaml").string()).has_value());
}

TEST_F(TempDir, AMismatchedSidecarIsIgnored)
{
  Write("map.png", "the image bytes");
  const std::string yaml = Write("map.yaml", "image: map.png\n");
  Sidecar s;
  s.epoch = "e1";
  s.map_sha256 = Sha256Hex("the image bytes");
  s.saved_pose = SavedPose{};
  ASSERT_TRUE(WriteSidecar(SidecarPath(yaml), s));
  EXPECT_TRUE(LoadMatchingSidecar(yaml).has_value());
  Write("map.png", "another image");
  EXPECT_FALSE(LoadMatchingSidecar(yaml).has_value());
  fs::remove(SidecarPath(yaml));
  EXPECT_FALSE(LoadMatchingSidecar(yaml).has_value());
}

TEST_F(TempDir, SessionRoundTrip)
{
  Session s;
  s.epoch = "feedbeef";
  s.gid = "01.0f,ab";
  s.word = Word::kHeld;
  s.version = 9;
  s.alignment = SomeAlignment();
  s.alignment->scale = 1.0123;
  const std::string path = (dir_ / "session.yaml").string();
  ASSERT_TRUE(WriteSession(path, s));
  const auto back = ReadSession(path);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->epoch, "feedbeef");
  EXPECT_EQ(back->gid, "01.0f,ab");
  EXPECT_EQ(back->word, Word::kHeld);
  EXPECT_EQ(back->version, 9u);
  ASSERT_TRUE(back->alignment.has_value());
  EXPECT_NEAR(back->alignment->scale, 1.0123, 1e-6);
  EXPECT_EQ(back->alignment->source, Source::kFit);

  Session empty;
  empty.epoch = "x";
  const auto e = ParseSession(SessionYaml(empty));
  ASSERT_TRUE(e.has_value());
  EXPECT_FALSE(e->alignment.has_value());
  EXPECT_FALSE(ParseSession("epoch: x\n").has_value());
}
