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

// R26: the local costmap's PNG (the cost as is, 255 unknown, row 0 the top), the pooling (lethal
// survives, at most 400 cells a side, never over 32 768 B), the window's origin in the map frame,
// and when it is sent (at most once per gap, only on a change of the cells or the place).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/costmap_image.hpp"

using namespace husarion_ugv_map_gateway;

namespace
{

GreyImage Decode(const CostPng & png)
{
  const auto image = DecodeGreyPng(png.png);
  EXPECT_TRUE(image.has_value());
  return image.value_or(GreyImage{});
}

// A local costmap as Nav2 publishes it: obstacles from a LiDAR (lethal 100), inscribed (99) and
// the inflation ramp exp(-k (d - r_inscribed)) translated to 1..98, unknown (-1) beyond the
// sensor's reach. The window is `side` cells at `resolution`, the robot at its centre.
CostGrid RealisticCostmap(uint32_t side, double resolution, unsigned seed)
{
  const double inscribed = 0.3, inflation = 2.0, scaling = 2.0, reach = 3.5;
  CostGrid g;
  g.width = g.height = side;
  g.resolution = resolution;
  const double half = side * resolution / 2.0;
  std::vector<char> lethal(size_t(side) * side, 0);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.02);
  auto mark = [&](double x, double y) {
    const int c = int(std::floor((x + half) / resolution));
    const int r = int(std::floor((y + half) / resolution));
    if (c >= 0 && r >= 0 && c < int(side) && r < int(side) && std::hypot(x, y) < reach) {
      lethal[size_t(r) * side + c] = 1;
    }
  };
  // A corridor's two walls, a box, a pillar and scattered returns, sampled as LiDAR points.
  for (double t = -half; t < half; t += 0.01) {
    mark(t, 1.2 + noise(rng));
    mark(t, -1.4 + noise(rng));
  }
  for (double t = 0.0; t < 0.6; t += 0.01) {
    mark(1.0 + t, 0.4 + noise(rng));
    mark(1.0 + noise(rng), 0.4 + t * 0.5);
  }
  for (double a = 0.0; a < 2 * M_PI; a += 0.05) {
    mark(-1.5 + 0.15 * std::cos(a), -0.3 + 0.15 * std::sin(a));
  }
  std::uniform_real_distribution<double> anywhere(-half, half);
  for (int i = 0; i < 40; ++i) {
    mark(anywhere(rng), anywhere(rng));
  }
  // Distance to the nearest lethal cell (two-pass chamfer, in cells).
  const double inf = 1e9;
  std::vector<double> d(size_t(side) * side, inf);
  for (size_t i = 0; i < d.size(); ++i) {
    if (lethal[i]) {
      d[i] = 0.0;
    }
  }
  auto relax = [&](int r, int c, int dr, int dc, double w) {
    const int rr = r + dr, cc = c + dc;
    if (rr >= 0 && cc >= 0 && rr < int(side) && cc < int(side)) {
      d[size_t(r) * side + c] = std::min(d[size_t(r) * side + c], d[size_t(rr) * side + cc] + w);
    }
  };
  for (int r = 0; r < int(side); ++r) {
    for (int c = 0; c < int(side); ++c) {
      relax(r, c, -1, 0, 1.0), relax(r, c, 0, -1, 1.0);
      relax(r, c, -1, -1, M_SQRT2), relax(r, c, -1, 1, M_SQRT2);
    }
  }
  for (int r = int(side) - 1; r >= 0; --r) {
    for (int c = int(side) - 1; c >= 0; --c) {
      relax(r, c, 1, 0, 1.0), relax(r, c, 0, 1, 1.0);
      relax(r, c, 1, 1, M_SQRT2), relax(r, c, 1, -1, M_SQRT2);
    }
  }
  g.cells.assign(size_t(side) * side, 0);
  for (uint32_t r = 0; r < side; ++r) {
    for (uint32_t c = 0; c < side; ++c) {
      const size_t i = size_t(r) * side + c;
      const double x = (c + 0.5) * resolution - half, y = (r + 0.5) * resolution - half;
      const double dist = d[i] * resolution;
      int8_t v = 0;
      if (d[i] == 0.0) {
        v = 100;
      } else if (dist <= inscribed) {
        v = 99;
      } else if (dist <= inflation) {
        const double cost = 252.0 * std::exp(-scaling * (dist - inscribed));  // Nav2's 0..252
        v = cost < 1.0 ? 0 : int8_t(1 + (97 * (int(cost) - 1)) / 251);
      }
      if (std::hypot(x, y) > reach && v < 99) {
        v = -1;  // never seen
      }
      g.cells[i] = v;
    }
  }
  return g;
}

}  // namespace

TEST(CostmapImage, ValuesUnknownAndRowOrder)
{
  // 4 x 2: the bottom row (the smallest y) first in the grid, the top first in the PNG.
  CostGrid g;
  g.width = 4;
  g.height = 2;
  g.resolution = 0.05;
  g.cells = {0, 1, 50, 98, 99, 100, -1, 127};
  const auto png = EncodeCostmap(g);
  ASSERT_TRUE(png.has_value());
  EXPECT_EQ(png->poolings, 0);
  EXPECT_EQ(png->width, 4u);
  EXPECT_EQ(png->height, 2u);
  EXPECT_DOUBLE_EQ(png->resolution, 0.05);
  const auto image = Decode(*png);
  const std::vector<uint8_t> expect = {99, 100, 255, 100,  // PNG row 0 = grid row 1 (the top)
                                       0,  1,   50,  98};
  EXPECT_EQ(image.pixels, expect);
  EXPECT_EQ(kCostUnknown, 255);
}

TEST(CostmapImage, PoolingKeepsLethalAndKnownBeatsUnknown)
{
  CostGrid g;
  g.width = 5;
  g.height = 3;
  g.resolution = 0.1;
  g.cells = {
    0,  0,   -1, -1, 7,   // row 0
    0,  100, -1, -1, -1,  // row 1
    -1, -1,  3,  -1, -1,  // row 2
  };
  const CostGrid p = PoolCost(g);
  EXPECT_EQ(p.width, 3u);
  EXPECT_EQ(p.height, 2u);
  EXPECT_DOUBLE_EQ(p.resolution, 0.2);
  const std::vector<int8_t> expect = {100, -1, 7, -1, 3, -1};
  EXPECT_EQ(p.cells, expect);
}

TEST(CostmapImage, AWindowOver400CellsIsPooledAndOneLethalCellSurvives)
{
  CostGrid g;
  g.width = 1000;
  g.height = 300;
  g.resolution = 0.02;
  g.cells.assign(size_t(g.width) * g.height, 0);
  g.cells[size_t(123) * g.width + 777] = 100;
  const auto png = EncodeCostmap(g);
  ASSERT_TRUE(png.has_value());
  EXPECT_EQ(png->poolings, 2);
  EXPECT_EQ(png->width, 250u);
  EXPECT_EQ(png->height, 75u);
  EXPECT_DOUBLE_EQ(png->resolution, 0.08);
  const auto image = Decode(*png);
  // grid (777, 123) is pooled cell (194, 30): PNG row 75 - 1 - 30.
  EXPECT_EQ(image.pixels[size_t(44) * 250 + 194], 100);
  EXPECT_EQ(std::count(image.pixels.begin(), image.pixels.end(), 100), 1);
}

TEST(CostmapImage, NeverOverTheCap)
{
  // Noise does not compress: the encoder pools until it fits, never sends it over 32 768 B.
  CostGrid g;
  g.width = g.height = 400;
  g.resolution = 0.02;
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> any(-1, 100);
  for (size_t i = 0; i < size_t(400) * 400; ++i) {
    g.cells.push_back(int8_t(any(rng)));
  }
  const auto png = EncodeCostmap(g);
  ASSERT_TRUE(png.has_value());
  EXPECT_LE(png->png.size(), 32768u);
  EXPECT_GE(png->poolings, 1);
  EXPECT_LE(png->width, 400u);
  // A cap nothing fits: nothing.
  EXPECT_FALSE(EncodeCostmap(g, 400, 10).has_value());
}

TEST(CostmapImage, MalformedGridsAreNotEncoded)
{
  CostGrid g;
  EXPECT_FALSE(EncodeCostmap(g).has_value());
  g.width = 3;
  g.height = 3;
  g.resolution = 0.05;
  g.cells.assign(8, 0);  // one short
  EXPECT_FALSE(EncodeCostmap(g).has_value());
  g.cells.assign(9, 0);
  g.resolution = 0.0;
  EXPECT_FALSE(EncodeCostmap(g).has_value());
}

TEST(CostmapImage, RealisticWindowsFitAsSent)
{
  // The sizes Nav2's windows give: 100 x 100 (2 m at 2 cm) and 400 x 400 (the stack's 8 m at
  // 2 cm, nav2_params.yaml's local_costmap). Printed for the spec's size column.
  for (const uint32_t side : {100u, 400u}) {
    const CostGrid g = RealisticCostmap(side, 0.02, side);
    const auto png = EncodeCostmap(g);
    ASSERT_TRUE(png.has_value());
    EXPECT_LE(png->png.size(), 32768u);
    EXPECT_EQ(png->poolings, 0) << side;
    const auto image = Decode(*png);
    EXPECT_GT(std::count(image.pixels.begin(), image.pixels.end(), 100), 0);
    std::cout << "[ costmap ] " << side << " x " << side << " realistic window: " << png->png.size()
              << " B, " << png->poolings << " poolings\n";
    RecordProperty("png_bytes_" + std::to_string(side), int(png->png.size()));
  }
}

TEST(CostmapImage, OriginThroughAYawedOdomToMap)
{
  // odom sits at (2, 1) in the map, turned by 90 degrees; the window's origin is (-4, -4) in
  // odom with no turn of its own.
  const Pose2 odom_in_map{2.0, 1.0, M_PI / 2};
  const Pose2 origin = Compose(odom_in_map, {-4.0, -4.0, 0.0});
  EXPECT_NEAR(origin.x, 2.0 + 4.0, 1e-12);
  EXPECT_NEAR(origin.y, 1.0 - 4.0, 1e-12);
  EXPECT_NEAR(origin.yaw, M_PI / 2, 1e-12);
  // A turned origin adds its yaw; the sum is wrapped.
  const Pose2 wrapped = Compose({0.0, 0.0, 3.0}, {1.0, 0.0, 0.5});
  EXPECT_NEAR(wrapped.x, std::cos(3.0), 1e-12);
  EXPECT_NEAR(wrapped.y, std::sin(3.0), 1e-12);
  EXPECT_NEAR(wrapped.yaw, 3.5 - 2 * M_PI, 1e-12);
  // The identity changes nothing.
  const Pose2 same = Compose({}, {-4.0, -3.0, 0.25});
  EXPECT_DOUBLE_EQ(same.x, -4.0);
  EXPECT_DOUBLE_EQ(same.y, -3.0);
  EXPECT_DOUBLE_EQ(same.yaw, 0.25);
}

TEST(CostmapGate, AtMostOncePerGap)
{
  CostmapGate gate(1.0, 0.005);
  EXPECT_TRUE(gate.Due(10.0));
  gate.Sent(10.0, 1, {});
  EXPECT_FALSE(gate.Due(10.5));  // Nav2's 2 Hz: every other one waits
  EXPECT_FALSE(gate.Due(10.99));
  EXPECT_TRUE(gate.Due(11.0));
}

TEST(CostmapGate, AStillSceneIsSentEvery2sAndAChangeWithinTheSpacing)
{
  CostmapGate gate(1.0, 0.005, 2.0);
  const Pose2 at{-4.0, -4.0, 0.0};
  ASSERT_TRUE(gate.ShouldSend(10.0, 42, at, 11.3));
  gate.Sent(10.0, 42, at);
  // Nav2's 2 Hz windows, all the same: the next goes out 2 s later, not sooner.
  for (double t = 10.5; t < 12.0; t += 0.5) {
    EXPECT_FALSE(gate.ShouldSend(t, 42, at, 11.3)) << t;
  }
  EXPECT_TRUE(gate.ShouldSend(12.0, 42, at, 11.3));
  gate.Sent(12.0, 42, at);
  // A change 0.5 s later waits for the 1 s spacing, then goes at once.
  EXPECT_FALSE(gate.ShouldSend(12.5, 43, at, 11.3));
  EXPECT_TRUE(gate.ShouldSend(13.0, 43, at, 11.3));
  gate.Sent(13.0, 43, at);
  // Still again: 2 s after the change, not after the resend before it.
  EXPECT_FALSE(gate.ShouldSend(14.5, 43, at, 11.3));
  EXPECT_TRUE(gate.ShouldSend(15.0, 43, at, 11.3));
}

TEST(CostmapGate, OnlyOnAChangeOfTheCellsOrThePlace)
{
  CostmapGate gate(1.0, 0.005);
  const Pose2 at{-4.0, -4.0, 0.1};
  EXPECT_TRUE(gate.Changed(42, at, 11.3));  // nothing sent yet
  gate.Sent(10.0, 42, at);
  EXPECT_FALSE(gate.Changed(42, at, 11.3));                          // the same window
  EXPECT_FALSE(gate.Changed(42, {-4.003, -4.0, 0.1}, 11.3));         // within the tolerance
  EXPECT_TRUE(gate.Changed(43, at, 11.3));                           // other cells
  EXPECT_TRUE(gate.Changed(42, {-3.98, -4.0, 0.1}, 11.3));           // rolled one cell
  EXPECT_TRUE(gate.Changed(42, {-4.0, -4.0, 0.1 + 0.001}, 11.3));    // turned: 11 mm at the corner
  EXPECT_FALSE(gate.Changed(42, {-4.0, -4.0, 0.1 + 0.0001}, 11.3));  // 1 mm at the corner
}
