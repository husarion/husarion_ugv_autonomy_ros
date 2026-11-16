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

// §11.2: the PNG budget and the reported resolution (§5.1, D18).

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/map_image.hpp"

using namespace husarion_ugv_map_gateway;

TEST(MapImage, MapServerConvention)
{
  const std::vector<int8_t> data = {-1, 0, 25, 26, 64, 65, 100, 50};
  const GreyGrid g = ClassifyGrid(data, 8, 1, 0.05);
  const std::vector<uint8_t> expect = {kUnknown, kFree,     kFree,     kUnknown,
                                       kUnknown, kOccupied, kOccupied, kUnknown};
  EXPECT_EQ(g.cells, expect);
  EXPECT_EQ(kOccupied, 0);
  EXPECT_EQ(kFree, 254);
  EXPECT_EQ(kUnknown, 205);
}

TEST(MapImage, RowZeroIsTheTop)
{
  // 3 x 2 grid; the occupied cell is at the largest y (grid row 1), column 0.
  std::vector<int8_t> data(6, 0);
  data[3] = 100;
  const auto png = EncodeWithinBudget(ClassifyGrid(data, 3, 2, 0.1));
  ASSERT_TRUE(png.has_value());
  const auto image = DecodeGreyPng(png->png);
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->pixels[0], kOccupied);  // PNG row 0, column 0
  EXPECT_EQ(image->pixels[3], kFree);      // PNG row 1 = grid row 0
  EXPECT_EQ(png->halvings, 0);
  EXPECT_DOUBLE_EQ(png->resolution, 0.1);
}

TEST(MapImage, DownsampleKeepsWalls)
{
  // A one-cell wall survives every halving; free beats unknown; odd sides are padded.
  GreyGrid g;
  g.width = 9;
  g.height = 7;
  g.resolution = 0.05;
  g.cells.assign(63, kUnknown);
  for (uint32_t row = 0; row < 7; ++row) {
    g.cells[row * 9 + 4] = kOccupied;
  }
  g.cells[0] = kFree;
  GreyGrid d = Downsample(g);
  EXPECT_EQ(d.width, 5u);
  EXPECT_EQ(d.height, 4u);
  EXPECT_DOUBLE_EQ(d.resolution, 0.1);
  EXPECT_EQ(d.cells[0], kFree);
  for (uint32_t row = 0; row < d.height; ++row) {
    EXPECT_EQ(d.cells[row * d.width + 2], kOccupied) << row;
  }
  d = Downsample(Downsample(d));
  EXPECT_EQ(d.width, 2u);
  EXPECT_EQ(d.cells[0], kOccupied);
}

TEST(MapImage, BudgetHalvesUntilItFits)
{
  // A large grid of noise does not compress: it must be halved until under the budget, and the
  // reported resolution is the source's times 2^halvings.
  const uint32_t w = 3000, h = 2500;
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> pick(0, 2);
  std::vector<int8_t> data(size_t(w) * h);
  for (auto & v : data) {
    const int p = pick(rng);
    v = p == 0 ? -1 : (p == 1 ? 0 : 100);
  }
  const auto png = EncodeWithinBudget(ClassifyGrid(data, w, h, 0.05), 190000, 2048, 200000);
  ASSERT_TRUE(png.has_value());
  EXPECT_LE(png->png.size(), 190000u);
  EXPECT_LE(png->width, 2048u);
  EXPECT_LE(png->height, 2048u);
  EXPECT_GE(png->halvings, 1);
  EXPECT_DOUBLE_EQ(png->resolution, 0.05 * (1 << png->halvings));
  const auto image = DecodeGreyPng(png->png);
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->width, png->width);
  EXPECT_EQ(image->height, png->height);
  EXPECT_EQ(png->width, (w + (1u << png->halvings) - 1) >> png->halvings);
}

TEST(MapImage, SideLimitHalvesAnEasyGrid)
{
  // Compresses to almost nothing, but a side of 3000 cells is over 2048.
  std::vector<int8_t> data(size_t(3000) * 10, 0);
  const auto png = EncodeWithinBudget(ClassifyGrid(data, 3000, 10, 0.05));
  ASSERT_TRUE(png.has_value());
  EXPECT_EQ(png->halvings, 1);
  EXPECT_EQ(png->width, 1500u);
  EXPECT_EQ(png->height, 5u);
  EXPECT_DOUBLE_EQ(png->resolution, 0.1);
}

TEST(MapImage, NeverOverTheCap)
{
  std::vector<int8_t> data(size_t(64) * 64, 0);
  // A budget nothing meets: no PNG at all rather than one over the cap.
  EXPECT_FALSE(EncodeWithinBudget(ClassifyGrid(data, 64, 64, 0.05), 10, 2048, 10).has_value());
  EXPECT_FALSE(EncodeWithinBudget(ClassifyGrid({}, 0, 0, 0.05)).has_value());
}
