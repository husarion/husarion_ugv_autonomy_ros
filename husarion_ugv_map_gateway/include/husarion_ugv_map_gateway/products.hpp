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

#ifndef HUSARION_UGV_MAP_GATEWAY_PRODUCTS_HPP_
#define HUSARION_UGV_MAP_GATEWAY_PRODUCTS_HPP_

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace husarion_ugv_map_gateway
{

// Publishing under a lane's `max_hz` (§5.1 "Below the caps"): a message offered sooner than
// `min_gap` after the last one sent is held back and sent once the gap allows, never dropped;
// when several wait, only the newest is kept.
template <typename T>
class Spacer
{
public:
  explicit Spacer(double min_gap) : min_gap_(min_gap) {}

  // Returns the message to send now, or nullopt if it was held back.
  std::optional<T> Offer(T msg, double now)
  {
    if (!last_sent_ || now - *last_sent_ >= min_gap_) {
      pending_.reset();
      last_sent_ = now;
      return msg;
    }
    pending_ = std::move(msg);
    return std::nullopt;
  }

  // The held-back message, once its time has come.
  std::optional<T> Poll(double now)
  {
    if (pending_ && (!last_sent_ || now - *last_sent_ >= min_gap_)) {
      std::optional<T> out = std::move(pending_);
      pending_.reset();
      last_sent_ = now;
      return out;
    }
    return std::nullopt;
  }

  bool has_pending() const { return pending_.has_value(); }
  std::optional<double> last_sent() const { return last_sent_; }

private:
  double min_gap_;
  std::optional<double> last_sent_;
  std::optional<T> pending_;
};

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

// Douglas-Peucker: the indices of the points kept (the first and the last always).
std::vector<size_t> DouglasPeucker(const std::vector<Point2> & points, double tolerance);

// §5.1 the plan: Douglas-Peucker at `tolerance`; while more than `max_points` remain, the
// tolerance doubles, so the path keeps both ends and its shape instead of being cut short.
std::vector<size_t> SimplifyPath(
  const std::vector<Point2> & points, double tolerance = 0.05, size_t max_points = 200);

// §5.1 / §6: every float published is finite.
template <typename... Ts>
bool AllFinite(Ts... values)
{
  return (std::isfinite(static_cast<double>(values)) && ...);
}

// The scan rule (phase 2, M2a publishes it): at most `max_ranges` ranges (every k-th), and a
// range of inf or NaN becomes range_max + 1, out of range by the message's own rule.
struct ScanOut
{
  std::vector<float> ranges;
  int stride = 1;
};
ScanOut ClampScan(const std::vector<float> & ranges, float range_max, size_t max_ranges = 720);

// `clock_offset_s` (§5.2, §6): this computer's clock minus the Pi's stamp on the newest fixes.
// The smallest (local - stamp) over the last few samples, which removes most of the transport
// delay; nullopt (`unknown`) when no fix arrived for `max_age` seconds.
class ClockOffset
{
public:
  explicit ClockOffset(double max_age = 5.0, size_t window = 10)
  : max_age_(max_age), window_(window)
  {
  }
  void Add(double local_now, double stamp);
  std::optional<double> Value(double local_now) const;

private:
  double max_age_;
  size_t window_;
  std::deque<std::pair<double, double>> samples_;  // (local time, local - stamp)
};

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_PRODUCTS_HPP_
