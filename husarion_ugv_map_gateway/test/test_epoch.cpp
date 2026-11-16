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

// §11.2 the epoch: new on a changed publisher GID, on another map_file_name, at a start with no
// saved pair and no matching session file; unchanged while the grid's origin, size and content
// change during mapping; restored from the session file when the gateway restarts alone; the
// sidecar's on a saved pair, also across a map_server restart on the same map.

#include <gtest/gtest.h>

#include <string>

#include "husarion_ugv_map_gateway/epoch.hpp"

using namespace husarion_ugv_map_gateway;
using Why = EpochTracker::Why;

namespace
{

EpochTracker Make()
{
  static int n = 0;
  return EpochTracker(EpochTracker::Params(), [] { return "e" + std::to_string(++n); });
}

Sidecar SavedPair(const std::string & epoch)
{
  Sidecar s;
  s.epoch = epoch;
  s.map_sha256 = "x";
  return s;
}

}  // namespace

TEST(Epoch, NewAtAStartWithNothing)
{
  EpochTracker e = Make();
  e.Start("", std::nullopt, std::nullopt, 0.0);
  ASSERT_TRUE(e.decided());
  EXPECT_EQ(e.why(), Why::kNew);
  const std::string first = e.epoch();
  EXPECT_FALSE(e.OnPublisher("gidA", 1.0));  // the first publisher binds; no change
  EXPECT_EQ(e.epoch(), first);
  EXPECT_EQ(e.gid(), "gidA");
}

TEST(Epoch, NewOnAChangedPublisher)
{
  EpochTracker e = Make();
  e.Start("", std::nullopt, std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  const std::string first = e.epoch();
  EXPECT_FALSE(e.OnPublisher("", 2.0));  // a gap while slam_toolbox restarts is no change
  EXPECT_EQ(e.epoch(), first);
  EXPECT_TRUE(e.OnPublisher("gidB", 3.0));
  EXPECT_NE(e.epoch(), first);
  EXPECT_EQ(e.why(), Why::kPublisherChanged);
}

TEST(Epoch, UnchangedWhileTheGridGrows)
{
  EpochTracker e = Make();
  e.Start("", std::nullopt, std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  const std::string first = e.epoch();
  for (uint64_t h = 1; h < 50; ++h) {  // origin, size and content change on every update
    EXPECT_FALSE(e.OnGrid(h * 7919, 1.0 + h));
    EXPECT_FALSE(e.OnPublisher("gidA", 1.0 + h));
    EXPECT_FALSE(e.OnTick(1.0 + h));
  }
  EXPECT_EQ(e.epoch(), first);
}

TEST(Epoch, RestoredFromTheSessionWhenTheGidMatches)
{
  Session s;
  s.epoch = "kept";
  s.gid = "gidA";
  s.word = Word::kAligned;
  s.version = 4;
  EpochTracker e = Make();
  e.Start("", std::nullopt, s, 0.0);
  EXPECT_FALSE(e.decided());  // waits for the publisher
  EXPECT_FALSE(e.OnPublisher("", 1.0));
  EXPECT_TRUE(e.OnPublisher("gidA", 2.0));
  EXPECT_EQ(e.epoch(), "kept");
  EXPECT_EQ(e.why(), Why::kSession);
  const auto restored = e.TakeRestored();
  ASSERT_TRUE(restored.has_value());
  EXPECT_EQ(restored->version, 4u);
  EXPECT_FALSE(e.TakeRestored().has_value());
}

TEST(Epoch, NewWhenTheSessionGidDiffers)
{
  Session s;
  s.epoch = "old";
  s.gid = "gidA";
  EpochTracker e = Make();
  e.Start("", std::nullopt, s, 0.0);
  EXPECT_TRUE(e.OnPublisher("gidB", 1.0));
  EXPECT_NE(e.epoch(), "old");
  EXPECT_EQ(e.why(), Why::kNew);
  EXPECT_FALSE(e.TakeRestored().has_value());
}

TEST(Epoch, NewWhenNoPublisherMatchesTheSessionInTime)
{
  Session s;
  s.epoch = "old";
  s.gid = "gidA";
  EpochTracker e = Make();
  e.Start("", std::nullopt, s, 0.0);
  EXPECT_FALSE(e.OnTick(29.0));
  EXPECT_TRUE(e.OnTick(30.0));
  EXPECT_NE(e.epoch(), "old");
  EXPECT_FALSE(e.OnPublisher("gidA", 31.0));  // binds to the new epoch
  EXPECT_EQ(e.gid(), "gidA");
}

TEST(Epoch, SidecarsEpochOnASavedPair)
{
  EpochTracker e = Make();
  e.Start("/maps/hall/map.yaml", SavedPair("saved1"), std::nullopt, 0.0);
  EXPECT_EQ(e.epoch(), "saved1");
  EXPECT_EQ(e.why(), Why::kSidecar);
  EXPECT_TRUE(e.on_saved_pair());
}

TEST(Epoch, SavedPairSurvivesAMapServerRestartOnTheSameMap)
{
  EpochTracker e = Make();
  e.Start("/maps/hall/map.yaml", SavedPair("saved1"), std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  e.OnGrid(42, 1.5);
  EXPECT_FALSE(e.OnPublisher("", 2.0));
  EXPECT_FALSE(e.OnPublisher("gidB", 3.0));  // pending: the new publisher's grid decides
  EXPECT_FALSE(e.OnGrid(42, 3.5));
  EXPECT_EQ(e.epoch(), "saved1");
  EXPECT_EQ(e.gid(), "gidB");
}

TEST(Epoch, SavedPairGetsANewEpochWhenTheNewPublisherServesAnotherMap)
{
  EpochTracker e = Make();
  e.Start("/maps/hall/map.yaml", SavedPair("saved1"), std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  e.OnGrid(42, 1.5);
  EXPECT_FALSE(e.OnPublisher("gidB", 3.0));
  EXPECT_TRUE(e.OnGrid(43, 3.5));
  EXPECT_NE(e.epoch(), "saved1");
  EXPECT_EQ(e.why(), Why::kPublisherChanged);
}

TEST(Epoch, SavedPairGetsANewEpochWhenTheNewPublisherSendsNothing)
{
  EpochTracker e = Make();
  e.Start("/maps/hall/map.yaml", SavedPair("saved1"), std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  e.OnGrid(42, 1.5);
  e.OnPublisher("gidB", 3.0);
  EXPECT_FALSE(e.OnTick(12.0));
  EXPECT_TRUE(e.OnTick(13.0));
  EXPECT_NE(e.epoch(), "saved1");
}

TEST(Epoch, NewOnAnotherMapFileName)
{
  EpochTracker e = Make();
  e.Start("/maps/a/map.yaml", SavedPair("A"), std::nullopt, 0.0);
  e.OnPublisher("gidA", 1.0);
  EXPECT_FALSE(e.OnMapFile("/maps/a/map.yaml", SavedPair("A"), 2.0));
  EXPECT_TRUE(e.OnMapFile("/maps/b/map.yaml", SavedPair("B"), 2.0));
  EXPECT_EQ(e.epoch(), "B");
  EXPECT_EQ(e.why(), Why::kMapFileChanged);
  EXPECT_TRUE(e.OnMapFile("/maps/c/map.yaml", std::nullopt, 3.0));  // no sidecar: a new one
  EXPECT_NE(e.epoch(), "B");
  EXPECT_FALSE(e.on_saved_pair());
  EXPECT_EQ(e.gid(), "gidA");  // map_server can load another map without a restart
}

TEST(Epoch, AMapWithoutItsSidecarGetsANewEpoch)
{
  EpochTracker e = Make();
  e.Start("/maps/a/map.yaml", std::nullopt, std::nullopt, 0.0);
  EXPECT_EQ(e.why(), Why::kNew);
  EXPECT_FALSE(e.on_saved_pair());
  e.OnPublisher("gidA", 1.0);
  e.OnGrid(42, 1.5);
  EXPECT_TRUE(e.OnPublisher("gidB", 2.0));  // no same-map rule without the sidecar
}
