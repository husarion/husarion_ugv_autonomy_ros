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

#include "husarion_ugv_map_gateway/costmap_image.hpp"

#include <algorithm>
#include <cmath>

#include "husarion_ugv_map_gateway/codec.hpp"

namespace husarion_ugv_map_gateway
{

CostGrid PoolCost(const CostGrid & grid)
{
  CostGrid out;
  out.width = (grid.width + 1) / 2;
  out.height = (grid.height + 1) / 2;
  out.resolution = grid.resolution * 2.0;
  out.cells.assign(size_t(out.width) * out.height, -1);
  for (uint32_t row = 0; row < out.height; ++row) {
    for (uint32_t col = 0; col < out.width; ++col) {
      int8_t best = -1;  // unknown is the lowest value, so it loses to any known cost
      for (uint32_t dr = 0; dr < 2; ++dr) {
        for (uint32_t dc = 0; dc < 2; ++dc) {
          const uint32_t r = 2 * row + dr;
          const uint32_t c = 2 * col + dc;
          if (r < grid.height && c < grid.width) {
            best = std::max(best, grid.cells[size_t(r) * grid.width + c]);
          }
        }
      }
      out.cells[size_t(row) * out.width + col] = best;
    }
  }
  return out;
}

std::vector<uint8_t> CostImageRows(const CostGrid & grid)
{
  std::vector<uint8_t> rows(size_t(grid.width) * grid.height);
  for (uint32_t row = 0; row < grid.height; ++row) {
    const size_t src = size_t(grid.height - 1 - row) * grid.width;
    for (uint32_t col = 0; col < grid.width; ++col) {
      const int v = grid.cells[src + col];
      rows[size_t(row) * grid.width + col] = v < 0 ? kCostUnknown : uint8_t(std::min(v, 100));
    }
  }
  return rows;
}

std::optional<CostPng> EncodeCostmap(
  const CostGrid & grid, uint32_t max_side, size_t cap, int max_poolings)
{
  if (
    grid.width == 0 || grid.height == 0 || grid.cells.size() != size_t(grid.width) * grid.height ||
    !(grid.resolution > 0.0)) {
    return std::nullopt;
  }
  CostGrid current = grid;
  for (int poolings = 0; poolings <= max_poolings; ++poolings) {
    if (current.width <= max_side && current.height <= max_side) {
      CostPng out;
      out.png = EncodeGreyPng(CostImageRows(current), current.width, current.height);
      if (out.png.size() <= cap) {
        out.width = current.width;
        out.height = current.height;
        out.resolution = current.resolution;
        out.poolings = poolings;
        return out;
      }
    }
    if (current.width == 1 && current.height == 1) {
      break;
    }
    current = PoolCost(current);
  }
  return std::nullopt;
}

Pose2 Compose(const Pose2 & frame, const Pose2 & local)
{
  const double c = std::cos(frame.yaw), s = std::sin(frame.yaw);
  Pose2 out;
  out.x = frame.x + c * local.x - s * local.y;
  out.y = frame.y + s * local.x + c * local.y;
  out.yaw = std::atan2(std::sin(frame.yaw + local.yaw), std::cos(frame.yaw + local.yaw));
  return out;
}

bool CostmapGate::Changed(uint64_t cells_hash, const Pose2 & origin, double extent) const
{
  if (!last_sent_ || cells_hash != hash_) {
    return true;
  }
  const double moved = std::hypot(origin.x - origin_.x, origin.y - origin_.y);
  const double turned =
    std::abs(std::atan2(std::sin(origin.yaw - origin_.yaw), std::cos(origin.yaw - origin_.yaw)));
  return moved > tol_ || turned * extent > tol_;
}

void CostmapGate::Sent(double now, uint64_t cells_hash, const Pose2 & origin)
{
  last_sent_ = now;
  hash_ = cells_hash;
  origin_ = origin;
}

}  // namespace husarion_ugv_map_gateway
