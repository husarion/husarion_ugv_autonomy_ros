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

// The saved-map preview behind ~/map_preview: map_saver's files back to a grid.

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "husarion_ugv_map_gateway/preview.hpp"

using namespace husarion_ugv_map_gateway;
namespace fs = std::filesystem;

namespace
{

// map_saver's own yaml: with free_thresh 0.196 its unknown grey (205) stays unknown.
fs::path SavedPair(const std::string & image, const std::string & name = "map.pgm")
{
  const fs::path dir = fs::temp_directory_path() / ("preview_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  std::ofstream(dir / name, std::ios::binary) << image;
  std::ofstream(dir / "map.yaml") << "image: " << name
                                  << "\nmode: trinary\nresolution: 0.05\n"
                                     "origin: [-1.5, 2.0, 0]\nnegate: 0\n"
                                     "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
  return dir / "map.yaml";
}

std::string Pgm(uint32_t w, uint32_t h, const std::vector<uint8_t> & px)
{
  return "P5\n# CREATOR: map_saver\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n" +
         std::string(px.begin(), px.end());
}

}  // namespace

TEST(Preview, DecodesMapSaversPgm)
{
  const auto image = DecodePgm(Pgm(3, 2, {0, 254, 205, 1, 2, 3}));
  ASSERT_TRUE(image);
  EXPECT_EQ(image->width, 3u);
  EXPECT_EQ(image->height, 2u);
  EXPECT_EQ(image->pixels, (std::vector<uint8_t>{0, 254, 205, 1, 2, 3}));
}

TEST(Preview, RefusesWhatIsNotAnEightBitBinaryPgm)
{
  EXPECT_FALSE(DecodePgm("P2\n1 1\n255\n0"));
  EXPECT_FALSE(DecodePgm("P5\n2 2\n65535\n...."));
  EXPECT_FALSE(DecodePgm("P5\n4 4\n255\nshort"));
  EXPECT_FALSE(DecodePgm(""));
}

TEST(Preview, LoadsAPairInOccupancyGridOrder)
{
  // top row: occupied, free; bottom row: unknown, free
  const auto saved = LoadSavedMap(SavedPair(Pgm(2, 2, {0, 254, 205, 254})).string());
  ASSERT_TRUE(saved);
  EXPECT_DOUBLE_EQ(saved->grid.resolution, 0.05);
  EXPECT_DOUBLE_EQ(saved->origin_x, -1.5);
  EXPECT_DOUBLE_EQ(saved->origin_y, 2.0);
  // grid row 0 is the image's bottom row
  EXPECT_EQ(saved->grid.cells, (std::vector<uint8_t>{kUnknown, kFree, kOccupied, kFree}));
}

TEST(Preview, LoadsAPngPair)
{
  // map_saver's image_format png, as the map gateway's saves write it
  const auto png = EncodeGreyPng({0, 254, 205, 254}, 2, 2);
  const auto saved = LoadSavedMap(SavedPair(png, "map.png").string());
  ASSERT_TRUE(saved);
  EXPECT_EQ(saved->grid.cells, (std::vector<uint8_t>{kUnknown, kFree, kOccupied, kFree}));
}

TEST(Preview, MissingOrBrokenPairsGiveNothing)
{
  EXPECT_FALSE(LoadSavedMap("/nonexistent/map.yaml"));
  EXPECT_FALSE(LoadSavedMap(SavedPair("not an image").string()));
}

TEST(Preview, ShrinksUntilBothSidesFitAndKeepsWalls)
{
  GreyGrid grid;
  grid.width = 1000;
  grid.height = 300;
  grid.resolution = 0.05;
  grid.cells.assign(size_t(grid.width) * grid.height, kFree);
  grid.cells[150 * 1000 + 500] = kOccupied;
  const GreyGrid small = ShrinkToSide(grid, 256);
  EXPECT_LE(small.width, 256u);
  EXPECT_LE(small.height, 256u);
  EXPECT_DOUBLE_EQ(small.resolution, 0.05 * 4);
  EXPECT_NE(std::find(small.cells.begin(), small.cells.end(), kOccupied), small.cells.end());
  EXPECT_EQ(ShrinkToSide(grid, 2000).width, 1000u);
}
