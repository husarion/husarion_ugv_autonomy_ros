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

#ifndef HUSARION_UGV_MAP_GATEWAY_MAP_IMAGE_HPP_
#define HUSARION_UGV_MAP_GATEWAY_MAP_IMAGE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace husarion_ugv_map_gateway
{

// map_server's trinary convention (spec §5.1).
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 254;
constexpr uint8_t kUnknown = 205;

// A grid already reduced to the three grey values, in OccupancyGrid order: cell (col, row) at
// index row * width + col, row 0 at the smallest y.
struct GreyGrid
{
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  std::vector<uint8_t> cells;
};

// OccupancyGrid data to grey: >= occupied_thresh is occupied, 0..free_thresh is free, the rest
// (-1 and the band between) unknown. The thresholds are map_saver's (0.65, 0.25) in percent.
GreyGrid ClassifyGrid(
  const std::vector<int8_t> & data, uint32_t width, uint32_t height, double resolution,
  int occupied_thresh = 65, int free_thresh = 25);

// Half the resolution: each 2 x 2 block becomes occupied if any cell is (walls survive), else
// free if any cell is, else unknown. An odd side is padded with unknown. The lower-left corner
// stays where it was, so `info.origin` does not change.
GreyGrid Downsample(const GreyGrid & grid);

// The rows of the PNG: row 0 is the top (the largest y), as in a saved map.pgm.
std::vector<uint8_t> ImageRows(const GreyGrid & grid);

struct MapPng
{
  std::string png;
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  int halvings = 0;
};

// D18: encode, and while the PNG is over `budget` bytes or a side is over `max_side` cells, halve
// the resolution and try again. nullopt (nothing to publish) when it never fits under `cap`.
std::optional<MapPng> EncodeWithinBudget(
  const GreyGrid & grid, size_t budget = 190000, uint32_t max_side = 2048, size_t cap = 200000,
  int max_halvings = 16);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_MAP_IMAGE_HPP_
