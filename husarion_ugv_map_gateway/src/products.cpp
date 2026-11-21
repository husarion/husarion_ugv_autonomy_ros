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

#include "husarion_ugv_map_gateway/products.hpp"

#include <algorithm>
#include <limits>

namespace husarion_ugv_map_gateway
{

namespace
{

double SegmentDistance(const Point2 & p, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 <= 0.0) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

}  // namespace

std::vector<size_t> DouglasPeucker(const std::vector<Point2> & points, double tolerance)
{
  const size_t n = points.size();
  if (n <= 2) {
    std::vector<size_t> all(n);
    for (size_t i = 0; i < n; ++i) {
      all[i] = i;
    }
    return all;
  }
  std::vector<char> keep(n, 0);
  keep[0] = keep[n - 1] = 1;
  std::vector<std::pair<size_t, size_t>> stack = {{0, n - 1}};
  while (!stack.empty()) {
    const auto [first, last] = stack.back();
    stack.pop_back();
    double worst = -1.0;
    size_t at = first;
    for (size_t i = first + 1; i < last; ++i) {
      const double d = SegmentDistance(points[i], points[first], points[last]);
      if (d > worst) {
        worst = d;
        at = i;
      }
    }
    if (worst > tolerance) {
      keep[at] = 1;
      stack.push_back({first, at});
      stack.push_back({at, last});
    }
  }
  std::vector<size_t> out;
  for (size_t i = 0; i < n; ++i) {
    if (keep[i]) {
      out.push_back(i);
    }
  }
  return out;
}

std::vector<size_t> SimplifyPath(
  const std::vector<Point2> & points, double tolerance, size_t max_points)
{
  std::vector<size_t> kept = DouglasPeucker(points, tolerance);
  while (kept.size() > max_points && max_points >= 2) {
    tolerance *= 2.0;
    kept = DouglasPeucker(points, tolerance);
  }
  return kept;
}

ScanOut ClampScan(
  const std::vector<float> & ranges, float range_min, float range_max, size_t max_ranges)
{
  ScanOut out;
  if (max_ranges == 0) {
    return out;
  }
  out.stride = static_cast<int>((ranges.size() + max_ranges - 1) / max_ranges);
  out.stride = std::max(out.stride, 1);
  const float beyond = std::isfinite(range_max) ? range_max + 1.0f : 1.0e6f;
  for (size_t i = 0; i < ranges.size(); i += out.stride) {
    float nearest = beyond;
    for (size_t j = i; j < std::min(ranges.size(), i + size_t(out.stride)); ++j) {
      const float r = ranges[j];
      if (std::isfinite(r) && r >= range_min && r <= range_max && r < nearest) {
        nearest = r;
      }
    }
    out.ranges.push_back(nearest);
  }
  return out;
}

void ClockOffset::Add(double local_now, double stamp)
{
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    return;
  }
  samples_.emplace_back(local_now, local_now - stamp);
  while (samples_.size() > window_) {
    samples_.pop_front();
  }
}

std::optional<double> ClockOffset::Value(double local_now) const
{
  std::optional<double> best;
  for (const auto & [t, offset] : samples_) {
    if (local_now - t > max_age_) {
      continue;
    }
    best = best ? std::min(*best, offset) : offset;
  }
  return best;
}

}  // namespace husarion_ugv_map_gateway
