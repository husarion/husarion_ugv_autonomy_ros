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

#include "husarion_ugv_map_gateway/state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace husarion_ugv_map_gateway
{

namespace
{

std::string Num(double value, int decimals)
{
  if (!std::isfinite(value)) {
    value = 0.0;
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  std::string s = buf;
  if (s == "-0" || s.rfind("-0.", 0) == 0) {
    // "-0.000" reads as a negative zero to a strict parser; write it plain.
    bool zero = true;
    for (char c : s.substr(1)) {
      zero = zero && (c == '0' || c == '.');
    }
    if (zero) {
      s = s.substr(1);
    }
  }
  return s;
}

size_t CdrString(size_t at, const std::string & s)
{
  at = (at + 3) & ~size_t(3);  // uint32 length, aligned to 4
  return at + 4 + s.size() + 1;
}

}  // namespace

std::vector<std::pair<std::string, std::string>> StateValues(const StateInputs & in)
{
  const bool has = in.alignment.has_value();
  std::vector<std::pair<std::string, std::string>> v;
  v.emplace_back("schema", kStateSchema);
  v.emplace_back("frame", in.frame);
  v.emplace_back("epoch", in.epoch);
  v.emplace_back("align", WordName(in.word));
  v.emplace_back("align_version", std::to_string(in.version));
  v.emplace_back("lat0", Num(has ? in.alignment->geo.lat0 : 0.0, 9));
  v.emplace_back("lon0", Num(has ? in.alignment->geo.lon0 : 0.0, 9));
  v.emplace_back("yaw", Num(has ? in.alignment->geo.yaw : 0.0, 6));
  v.emplace_back("err_m", Num(in.err_m, 3));
  v.emplace_back("need_m", Num(in.word == Word::kAligning ? in.need_m : 0.0, 3));
  v.emplace_back("rms_m", Num(has ? in.alignment->rms_m : in.progress_rms_m, 3));
  v.emplace_back("extent_m", Num(has ? in.alignment->extent_m : in.progress_extent_m, 3));
  v.emplace_back("pairs", std::to_string(has ? in.alignment->pairs : in.progress_pairs));
  v.emplace_back("scale", Num(has ? in.alignment->scale : in.progress_scale, 4));
  v.emplace_back("fix", FixClassName(has ? in.alignment->fix : in.last_fix));
  v.emplace_back("fix_age_s", in.fix_age_s ? Num(std::max(0.0, *in.fix_age_s), 1) : "unknown");
  v.emplace_back("aligned_at", Num(has ? in.alignment->fitted_at : 0.0, 0));
  v.emplace_back("source", SourceName(has ? in.alignment->source : Source::kFit));
  v.emplace_back("mode", in.mode);
  v.emplace_back("pose", in.pose_ok ? "ok" : "none");
  v.emplace_back("nav2", in.nav2);
  v.emplace_back("collision_monitor", in.collision_monitor);
  if (in.laser) {
    v.emplace_back("laser_x", Num((*in.laser)[0], 3));
    v.emplace_back("laser_y", Num((*in.laser)[1], 3));
    v.emplace_back("laser_yaw", Num((*in.laser)[2], 4));
  }
  v.emplace_back("map_stamp", Num(in.map_stamp.value_or(0.0), 9));
  v.emplace_back("clock_offset_s", in.clock_offset_s ? Num(*in.clock_offset_s, 3) : "unknown");
  return v;
}

size_t DiagnosticStatusCdrSize(
  const std::string & name, const std::string & message, const std::string & hardware_id,
  const std::vector<std::pair<std::string, std::string>> & values)
{
  size_t at = 4;  // encapsulation header
  at += 1;        // level (byte)
  at = CdrString(at, name);
  at = CdrString(at, message);
  at = CdrString(at, hardware_id);
  at = ((at + 3) & ~size_t(3)) + 4;  // sequence length
  for (const auto & [key, value] : values) {
    at = CdrString(at, key);
    at = CdrString(at, value);
  }
  return at;
}

}  // namespace husarion_ugv_map_gateway
