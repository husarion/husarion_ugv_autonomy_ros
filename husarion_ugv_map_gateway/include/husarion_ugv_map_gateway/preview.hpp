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

#ifndef HUSARION_UGV_MAP_GATEWAY_PREVIEW_HPP_
#define HUSARION_UGV_MAP_GATEWAY_PREVIEW_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/map_image.hpp"

namespace husarion_ugv_map_gateway
{

// A binary (P5) 8-bit PGM, as map_saver writes it; nullopt for anything else.
std::optional<GreyImage> DecodePgm(const std::string & bytes);

struct SavedMap
{
  GreyGrid grid;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
};

// A saved pair's grid: map.yaml (resolution, origin, thresholds, negate) and the PGM or PNG it
// names, classified as map_server loads it and turned to OccupancyGrid row order.
std::optional<SavedMap> LoadSavedMap(const std::string & map_yaml);

// Halved (walls survive, see Downsample) until neither side is over `max_side` cells.
GreyGrid ShrinkToSide(const GreyGrid & grid, uint32_t max_side);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_PREVIEW_HPP_
