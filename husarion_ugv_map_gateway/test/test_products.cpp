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

// §11.2: every published float is finite (an inf scan range becomes range_max + 1); the spacing
// of `state` and `plan` (a change inside the window is sent late, not lost); clock_offset_s; the
// `state` values all parse and fit the lane's 4096 B.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "husarion_ugv_map_gateway/products.hpp"
#include "husarion_ugv_map_gateway/state.hpp"

using namespace husarion_ugv_map_gateway;

TEST(Spacer, StateIsSentLateNotLost)
{
  Spacer<int> s(0.6);  // state: messages at least 0.6 s apart
  EXPECT_EQ(s.Offer(1, 10.0), 1);
  EXPECT_FALSE(s.Offer(2, 10.1).has_value());  // held back
  EXPECT_FALSE(s.Offer(3, 10.2).has_value());  // replaces 2: the newest is sent
  EXPECT_FALSE(s.Poll(10.5).has_value());
  EXPECT_EQ(s.Poll(10.61), 3);
  EXPECT_FALSE(s.Poll(11.0).has_value());
  EXPECT_FALSE(s.has_pending());
  EXPECT_EQ(s.Offer(4, 11.3), 4);
}

TEST(Spacer, PlanAndTheEmptyPath)
{
  Spacer<std::string> s(1.25);  // plan: at most once per 1.25 s
  EXPECT_EQ(s.Offer("plan", 0.0), "plan");
  EXPECT_FALSE(s.Offer("", 0.3).has_value());  // the goal ended: the empty path waits
  EXPECT_FALSE(s.Poll(1.2).has_value());
  EXPECT_EQ(s.Poll(1.25), "");
}

TEST(Spacer, NeverFasterThanTheGap)
{
  Spacer<int> s(0.6);
  std::vector<double> sent;
  for (int i = 0; i < 1000; ++i) {
    const double t = i * 0.05;
    if (s.Offer(i, t)) {
      sent.push_back(t);
    }
    if (s.Poll(t + 0.025)) {
      sent.push_back(t + 0.025);
    }
  }
  for (size_t i = 1; i < sent.size(); ++i) {
    EXPECT_GE(sent[i] - sent[i - 1], 0.6 - 1e-9);
  }
  EXPECT_GT(sent.size(), 70u);  // and close to the cap: 50 s / 0.6 s
}

TEST(Plan, DouglasPeucker)
{
  std::vector<Point2> line;
  for (int i = 0; i <= 100; ++i) {
    line.push_back({i * 0.1, i * 0.05});
  }
  EXPECT_EQ(DouglasPeucker(line, 0.05), (std::vector<size_t>{0, 100}));
  std::vector<Point2> corner = {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
  EXPECT_EQ(DouglasPeucker(corner, 0.05), (std::vector<size_t>{0, 2, 4}));
  // A 4 cm wiggle is inside the 5 cm tolerance.
  std::vector<Point2> wiggle = {{0, 0}, {1, 0.04}, {2, 0}};
  EXPECT_EQ(DouglasPeucker(wiggle, 0.05).size(), 2u);
}

TEST(Plan, AtMost200PosesWithBothEnds)
{
  std::vector<Point2> zigzag;
  for (int i = 0; i < 2000; ++i) {
    zigzag.push_back({i * 0.05, (i % 2) * 0.3});
  }
  const auto kept = SimplifyPath(zigzag, 0.05, 200);
  EXPECT_LE(kept.size(), 200u);
  EXPECT_EQ(kept.front(), 0u);
  EXPECT_EQ(kept.back(), 1999u);
}

TEST(Finite, ScanClamp)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<float> ranges(1024, 5.0f);
  ranges[0] = inf;
  ranges[2] = nan;
  ranges[4] = -inf;
  ranges[6] = inf;
  ranges[7] = nan;
  ranges[8] = 0.1f;   // below range_min: not a reading
  ranges[9] = 25.0f;  // above range_max
  const ScanOut out = ClampScan(ranges, 0.5f, 20.0f, 720);
  EXPECT_LE(out.ranges.size(), 720u);
  EXPECT_EQ(out.stride, 2);
  for (float r : out.ranges) {
    EXPECT_TRUE(std::isfinite(r));
  }
  EXPECT_FLOAT_EQ(out.ranges[0], 5.0f);   // inf beside a reading: the reading
  EXPECT_FLOAT_EQ(out.ranges[1], 5.0f);   // NaN beside a reading
  EXPECT_FLOAT_EQ(out.ranges[3], 21.0f);  // inf and NaN: range_max + 1
  EXPECT_FLOAT_EQ(out.ranges[4], 21.0f);  // nothing in range
  EXPECT_FLOAT_EQ(out.ranges[5], 5.0f);
  EXPECT_TRUE(AllFinite(1.0, 2.0f, 3));
  EXPECT_FALSE(AllFinite(1.0, std::numeric_limits<double>::quiet_NaN()));
}

TEST(Scan, DecimationKeepsTheNearestObstacle)
{
  // 2048 beams (an Ouster's 2048 x 10 slice) to at most 720: stride 3. A one-beam pole at 0.9 m
  // in the middle of a run survives; so does each run's own minimum.
  std::vector<float> ranges(2048, 8.0f);
  ranges[1001] = 0.9f;
  for (size_t i = 0; i < 300; i += 3) {
    ranges[i + 2] = 3.0f;
  }
  const ScanOut out = ClampScan(ranges, 0.3f, 30.0f, 720);
  EXPECT_EQ(out.stride, 3);
  EXPECT_EQ(out.ranges.size(), 683u);
  EXPECT_LE(out.ranges.size(), 720u);
  EXPECT_FLOAT_EQ(out.ranges[1001 / 3], 0.9f);
  EXPECT_EQ(std::count(out.ranges.begin(), out.ranges.end(), 0.9f), 1);
  for (size_t k = 0; k < 100; ++k) {
    EXPECT_FLOAT_EQ(out.ranges[k], 3.0f) << k;
  }
  // Under the limit, nothing is merged.
  const ScanOut same = ClampScan(std::vector<float>(628, 4.0f), 0.7f, 20.0f, 720);
  EXPECT_EQ(same.stride, 1);
  EXPECT_EQ(same.ranges.size(), 628u);
  // Never more than the limit, whatever the count.
  for (size_t n : {719u, 720u, 721u, 1440u, 1441u, 4096u}) {
    EXPECT_LE(ClampScan(std::vector<float>(n, 1.0f), 0.1f, 10.0f, 720).ranges.size(), 720u) << n;
  }
}

TEST(Spacer, ScanAt4HzUnderTheCapOf5)
{
  // The slice at 10 Hz for 5 s; the gateway offers each one and polls every 50 ms.
  Spacer<int> s(0.25);
  std::vector<double> sent;
  int n = 0;
  for (int tick = 0; tick <= 100; ++tick) {
    const double now = 100.0 + tick * 0.05;
    if (tick % 2 == 0) {
      if (s.Offer(n++, now)) {
        sent.push_back(now);
      }
    }
    if (s.Poll(now)) {
      sent.push_back(now);
    }
  }
  for (size_t i = 1; i < sent.size(); ++i) {
    EXPECT_GE(sent[i] - sent[i - 1], 0.25 - 1e-9);  // never two inside the lane's 0.2 s
  }
  EXPECT_GE(sent.size(), 16u);  // about 4 Hz: 5 s at 0.25 to 0.3 s
  EXPECT_LE(sent.size(), 21u);
}

TEST(Clock, Offset)
{
  ClockOffset c;
  EXPECT_FALSE(c.Value(100.0).has_value());  // unknown with no fix
  // This computer 2.5 s ahead of the Pi; transport 20 to 80 ms.
  for (int i = 0; i < 10; ++i) {
    const double pi_stamp = 1000.0 + i;
    c.Add(pi_stamp + 2.5 + 0.08 - 0.006 * i, pi_stamp);
  }
  ASSERT_TRUE(c.Value(1012.0).has_value());
  EXPECT_NEAR(*c.Value(1012.0), 2.526, 1e-9);  // the smallest delay of the recent fixes
  EXPECT_FALSE(c.Value(1020.0).has_value());   // no fix for 5 s: unknown again
  ClockOffset behind;
  behind.Add(500.0, 500.4);  // 0.4 s behind
  EXPECT_NEAR(*behind.Value(500.0), -0.4, 1e-9);
  behind.Add(501.0, 0.0);  // a fix with no stamp says nothing
  EXPECT_NEAR(*behind.Value(501.0), -0.4, 1e-9);
}

namespace
{

StateInputs WorstCase()
{
  StateInputs in;
  in.frame = std::string(64, 'f');
  in.epoch = std::string(64, 'e');
  in.word = Word::kNotAligned;
  in.version = 4294967295u;
  Alignment a;
  a.geo = {-89.123456789, -179.123456789, -3.141593};
  a.rms_m = 99999.999;
  a.extent_m = 99999.999;
  a.pairs = 2000;
  a.scale = 1.5;
  a.fix = FixClass::kRtkFloat;
  a.fitted_at = 4102444800.0;
  in.alignment = a;
  in.err_m = 99999.999;
  in.need_m = 99999.999;
  in.fix_age_s = 99999999.9;
  in.mode = "localization";
  in.nav2 = "inactive";
  in.collision_monitor = "inactive";
  in.laser = std::array<double, 3>{-12.345, -12.345, -3.1416};
  in.map_stamp = 4102444800.123456789;
  in.clock_offset_s = -99999.999;
  return in;
}

}  // namespace

TEST(State, EveryValueParses)
{
  StateInputs in = WorstCase();
  in.word = Word::kAligning;
  in.alignment->rms_m = std::numeric_limits<double>::quiet_NaN();
  in.clock_offset_s = std::nullopt;
  in.fix_age_s = std::nullopt;
  const auto values = StateValues(in);
  const std::regex number("^-?[0-9]+(\\.[0-9]+)?$");
  const std::set<std::string> words = {
    "align", "fix", "source", "mode", "pose", "nav2", "collision_monitor", "frame", "epoch"};
  for (const auto & [k, v] : values) {
    if (words.count(k)) {
      continue;
    }
    if ((k == "clock_offset_s" || k == "fix_age_s") && v == "unknown") {
      continue;
    }
    EXPECT_TRUE(std::regex_match(v, number)) << k << "=" << v;
  }
  std::set<std::string> keys;
  for (const auto & kv : values) {
    keys.insert(kv.first);
  }
  for (const char * k : {"schema",        "frame",
                         "epoch",         "align",
                         "align_version", "lat0",
                         "lon0",          "yaw",
                         "err_m",         "need_m",
                         "rms_m",         "extent_m",
                         "pairs",         "scale",
                         "fix",           "fix_age_s",
                         "aligned_at",    "source",
                         "mode",          "pose",
                         "nav2",          "collision_monitor",
                         "laser_x",       "laser_y",
                         "laser_yaw",     "map_stamp",
                         "clock_offset_s"}) {
    EXPECT_TRUE(keys.count(k)) << k;
  }
}

TEST(State, Formats)
{
  StateInputs in;
  in.frame = "panther_0957/map";
  in.epoch = "0123456789abcdef";
  in.word = Word::kAligning;
  in.need_m = 37.0;
  in.clock_offset_s = 2.5;
  in.fix_age_s = 0.04;
  auto values = StateValues(in);
  auto get = [&](const std::string & key) {
    for (const auto & kv : values) {
      if (kv.first == key) {
        return kv.second;
      }
    }
    return std::string("<missing>");
  };
  EXPECT_EQ(get("schema"), "1");
  EXPECT_EQ(get("align"), "aligning");
  EXPECT_EQ(get("need_m"), "37.000");
  EXPECT_EQ(get("align_version"), "0");
  EXPECT_EQ(get("lat0"), "0.000000000");
  EXPECT_EQ(get("clock_offset_s"), "2.500");
  EXPECT_EQ(get("fix_age_s"), "0.0");
  EXPECT_EQ(get("pose"), "none");
  EXPECT_EQ(get("nav2"), "absent");
  EXPECT_EQ(get("collision_monitor"), "absent");
  EXPECT_EQ(get("fix"), "none");
  EXPECT_EQ(get("laser_x"), "<missing>");  // unknown until its transform is
  in.word = Word::kAligned;                // need_m is 0 outside aligning
  values = StateValues(in);
  EXPECT_EQ(get("need_m"), "0.000");
  in.clock_offset_s = -0.0001;  // no "-0.000"
  values = StateValues(in);
  EXPECT_EQ(get("clock_offset_s"), "0.000");
}

TEST(State, FitsTheLaneCap)
{
  const StateInputs in = WorstCase();
  const auto values = StateValues(in);
  const size_t size = DiagnosticStatusCdrSize(kStateName, "not_aligned", in.epoch, values);
  EXPECT_LT(size, 4096u);
  EXPECT_GT(size, 500u);  // sanity: the estimate counts every string
}
