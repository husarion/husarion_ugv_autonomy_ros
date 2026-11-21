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

#ifndef HUSARION_UGV_MAP_GATEWAY_COSTMAP_IMAGE_HPP_
#define HUSARION_UGV_MAP_GATEWAY_COSTMAP_IMAGE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace husarion_ugv_map_gateway
{

// The local costmap's PNG (spec R26, §5.1): the cost 0 to 100 as is, 255 for unknown.
constexpr uint8_t kCostUnknown = 255;

// A costmap in OccupancyGrid order (cell (col, row) at row * width + col, row 0 at the smallest
// y), with OccupancyGrid's values: -1 unknown, 0 to 100 the cost.
struct CostGrid
{
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  std::vector<int8_t> cells;
};

// Half the resolution: each 2 x 2 block takes its highest cost, so lethal survives and any known
// cost beats unknown (-1). An odd side is padded with unknown. The lower-left corner stays where
// it was, so the origin does not change.
CostGrid PoolCost(const CostGrid & grid);

// The PNG's rows, row 0 the top (the largest y), as in the map PNG: a cost below 0 is 255, one
// above 100 is 100.
std::vector<uint8_t> CostImageRows(const CostGrid & grid);

struct CostPng
{
  std::string png;
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  int poolings = 0;
};

// Pool by 2 while a side is over `max_side` cells or the PNG is over `cap` bytes. nullopt (nothing
// to publish) for an empty or malformed grid, or when it never fits.
std::optional<CostPng> EncodeCostmap(
  const CostGrid & grid, uint32_t max_side = 400, size_t cap = 32768, int max_poolings = 12);

// A pose in the plane.
struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// The pose `local`, given in a frame whose pose in the target frame is `frame`, in the target
// frame: the costmap's origin (in odom) through the odom -> map transform. The yaw is wrapped to
// (-pi, pi].
Pose2 Compose(const Pose2 & frame, const Pose2 & local);

// When to send the costmap (§5.1 "below the caps", R26): no sooner than `min_gap` seconds after
// the last one sent; then at once when the cells or the window's place in the map frame changed
// since then, and otherwise again `resend` seconds after the last one, so a page opened on a
// still scene gets a picture. The place changed when the origin moved by more than
// `move_tolerance` metres or the window's far corner turned by more than that.
class CostmapGate
{
public:
  CostmapGate(double min_gap, double move_tolerance, double resend = 2.0)
  : min_gap_(min_gap), tol_(move_tolerance), resend_(resend)
  {
  }

  bool Due(double now) const { return !last_sent_ || now - *last_sent_ >= min_gap_; }
  // `extent`: the window's diagonal in metres, the lever of a turn.
  bool Changed(uint64_t cells_hash, const Pose2 & origin, double extent) const;
  // Due, and either changed or due for its resend.
  bool ShouldSend(double now, uint64_t cells_hash, const Pose2 & origin, double extent) const
  {
    return Due(now) && (Changed(cells_hash, origin, extent) || now - *last_sent_ >= resend_);
  }
  void Sent(double now, uint64_t cells_hash, const Pose2 & origin);

private:
  double min_gap_;
  double tol_;
  double resend_;
  std::optional<double> last_sent_;
  uint64_t hash_ = 0;
  Pose2 origin_;
};

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_COSTMAP_IMAGE_HPP_
