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

#ifndef HUSARION_UGV_MAP_GATEWAY_STATE_HPP_
#define HUSARION_UGV_MAP_GATEWAY_STATE_HPP_

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "husarion_ugv_map_gateway/align.hpp"

namespace husarion_ugv_map_gateway
{

constexpr const char * kStateName = "cockpit/nav";
constexpr const char * kStateSchema = "1";

// Everything `cockpit/nav/state` carries (§5.2).
struct StateInputs
{
  std::string frame;
  std::string epoch;
  Word word = Word::kNotAligned;
  uint32_t version = 0;
  std::optional<Alignment> alignment;
  double err_m = 0.0;
  double need_m = 0.0;
  // The fit in progress, shown while there is no alignment.
  double progress_rms_m = 0.0;
  double progress_extent_m = 0.0;
  int progress_pairs = 0;
  double progress_scale = 1.0;
  FixClass last_fix = FixClass::kNone;
  std::optional<double> fix_age_s;
  std::string mode = "mapping";  // mapping, localization or amcl
  bool pose_ok = false;
  std::string nav2 = "absent";  // active, inactive or absent
  std::string collision_monitor = "absent";
  std::optional<std::array<double, 3>> laser;  // x, y, yaw on the base
  std::optional<double> map_stamp;
  std::optional<double> clock_offset_s;
};

// The `values` of the message, in a fixed order. Every value parses: numbers are finite decimals
// (non-finite inputs are written as 0), words are from their lists, and only `fix_age_s` and
// `clock_offset_s` can be the word `unknown`. lat0, lon0 and yaw are 0 while there is no
// alignment (align_version is then 0 or tells the page it has none). The laser keys are left out
// until its transform is known (phase 2).
std::vector<std::pair<std::string, std::string>> StateValues(const StateInputs & in);

// The serialized (CDR) size of a DiagnosticStatus with these strings, for the 4096 B lane cap.
size_t DiagnosticStatusCdrSize(
  const std::string & name, const std::string & message, const std::string & hardware_id,
  const std::vector<std::pair<std::string, std::string>> & values);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_STATE_HPP_
