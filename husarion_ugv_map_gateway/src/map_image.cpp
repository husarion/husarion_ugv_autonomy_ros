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

#include "husarion_ugv_map_gateway/map_image.hpp"

#include "husarion_ugv_map_gateway/codec.hpp"

namespace husarion_ugv_map_gateway
{

GreyGrid ClassifyGrid(
  const std::vector<int8_t> & data, uint32_t width, uint32_t height, double resolution,
  int occupied_thresh, int free_thresh)
{
  GreyGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  const size_t n = size_t(width) * height;
  grid.cells.assign(n, kUnknown);
  for (size_t i = 0; i < n && i < data.size(); ++i) {
    const int v = data[i];
    if (v >= occupied_thresh) {
      grid.cells[i] = kOccupied;
    } else if (v >= 0 && v <= free_thresh) {
      grid.cells[i] = kFree;
    }
  }
  return grid;
}

GreyGrid Downsample(const GreyGrid & grid)
{
  GreyGrid out;
  out.width = (grid.width + 1) / 2;
  out.height = (grid.height + 1) / 2;
  out.resolution = grid.resolution * 2.0;
  out.cells.assign(size_t(out.width) * out.height, kUnknown);
  for (uint32_t row = 0; row < out.height; ++row) {
    for (uint32_t col = 0; col < out.width; ++col) {
      bool occupied = false;
      bool free = false;
      for (uint32_t dr = 0; dr < 2; ++dr) {
        for (uint32_t dc = 0; dc < 2; ++dc) {
          const uint32_t r = 2 * row + dr;
          const uint32_t c = 2 * col + dc;
          if (r >= grid.height || c >= grid.width) {
            continue;
          }
          const uint8_t v = grid.cells[size_t(r) * grid.width + c];
          occupied = occupied || v == kOccupied;
          free = free || v == kFree;
        }
      }
      out.cells[size_t(row) * out.width + col] = occupied ? kOccupied : (free ? kFree : kUnknown);
    }
  }
  return out;
}

std::vector<uint8_t> ImageRows(const GreyGrid & grid)
{
  std::vector<uint8_t> rows(grid.cells.size());
  for (uint32_t row = 0; row < grid.height; ++row) {
    const size_t src = size_t(grid.height - 1 - row) * grid.width;
    std::copy(
      grid.cells.begin() + src, grid.cells.begin() + src + grid.width,
      rows.begin() + size_t(row) * grid.width);
  }
  return rows;
}

std::optional<MapPng> EncodeWithinBudget(
  const GreyGrid & grid, size_t budget, uint32_t max_side, size_t cap, int max_halvings)
{
  if (grid.width == 0 || grid.height == 0) {
    return std::nullopt;
  }
  GreyGrid current = grid;
  for (int halvings = 0; halvings <= max_halvings; ++halvings) {
    if (current.width <= max_side && current.height <= max_side) {
      MapPng out;
      out.png = EncodeGreyPng(ImageRows(current), current.width, current.height);
      if (out.png.size() <= budget && out.png.size() <= cap) {
        out.width = current.width;
        out.height = current.height;
        out.resolution = current.resolution;
        out.halvings = halvings;
        return out;
      }
    }
    if (current.width == 1 && current.height == 1) {
      break;
    }
    current = Downsample(current);
  }
  return std::nullopt;
}

}  // namespace husarion_ugv_map_gateway
