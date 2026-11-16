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

#ifndef HUSARION_UGV_MAP_GATEWAY_GEO_HPP_
#define HUSARION_UGV_MAP_GATEWAY_GEO_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace husarion_ugv_map_gateway
{

struct LatLon
{
  double lat = 0.0;  // degrees, WGS-84
  double lon = 0.0;
};

struct EastNorth
{
  double e = 0.0;  // metres
  double n = 0.0;
};

// The local tangent plane (ENU) of the WGS-84 ellipsoid at `ref`, height 0. Exact, no UTM, no
// flat-earth scale: geodetic -> ECEF -> rotate into ref's east/north/up, and the up part dropped.
EastNorth LatLonToEnu(const LatLon & ref, const LatLon & p);
// The inverse: the point of ref's tangent plane (up = 0) back to geodetic latitude/longitude.
LatLon EnuToLatLon(const LatLon & ref, const EastNorth & en);

// The geo-reference of `state` (§5.2): the map origin at (lat0, lon0) and map x at `yaw` radians
// counter-clockwise from east. map (x, y) -> east = cos(yaw) x - sin(yaw) y,
// north = sin(yaw) x + cos(yaw) y in the tangent plane at (lat0, lon0), then EnuToLatLon.
struct GeoRef
{
  double lat0 = 0.0;
  double lon0 = 0.0;
  double yaw = 0.0;
};

LatLon MapToLatLon(const GeoRef & ref, double x, double y);
// nullopt when the input is not finite or not a latitude/longitude.
std::optional<EastNorth> LatLonToMap(const GeoRef & ref, const LatLon & p);

// ---- Fixes and the fit (§5.4) ----

enum class FixClass { kNone = 0, kPlain = 1, kRtkFloat = 2, kRtkFixed = 3 };
const char * FixClassName(FixClass cls);
std::optional<FixClass> ParseFixClass(const std::string & name);
// The floor of `err_m` per class: 2 m plain, 0.5 m RTK float, 0.2 m RTK fixed.
struct ErrFloors
{
  double plain = 2.0;
  double rtk_float = 0.5;
  double rtk_fixed = 0.2;
};
double ErrFloor(FixClass cls, const ErrFloors & floors);

struct FixClassParams
{
  double rtk_fixed_max_sigma = 0.05;  // m: an RTK (GBAS) fix this good is `rtk_fixed`
  double rtk_float_max_sigma = 0.5;   // m: up to this `rtk_float`, above it `plain`
  double default_plain_sigma = 2.5;   // m, when the receiver reports no covariance
  double default_rtk_sigma = 0.5;     // m, a GBAS fix with no covariance: `rtk_float` at best
};

struct FixQuality
{
  FixClass cls = FixClass::kNone;
  double sigma = 0.0;  // horizontal 1-sigma, metres
};

// NavSatFix `status.status` and `position_covariance` (type, east and north variances) to a class.
// RTK classes need status GBAS (2), so a plain receiver that reports a small covariance stays
// `plain`. A covariance that is unknown, non-finite or not positive takes the defaults.
FixQuality ClassifyFix(
  int status, uint8_t covariance_type, double var_east, double var_north,
  const FixClassParams & params);

// §5.4: status < 0, a non-finite value or 0,0 is dropped.
bool UsableFix(int status, double lat, double lon);

struct Pair
{
  double x = 0.0;  // the antenna in the map frame at the fix's stamp
  double y = 0.0;
  double e = 0.0;  // the fix in the epoch's local east-north metres
  double n = 0.0;
  double sigma = 1.0;
  FixClass cls = FixClass::kPlain;
};

struct RigidFit
{
  bool valid = false;
  double yaw = 0.0;  // east-north = R(yaw) * map + t
  double tx = 0.0;
  double ty = 0.0;
  double rms = 0.0;     // RMS of the inliers' residuals, metres
  double scale = 1.0;   // the similarity fit's scale, a diagnostic only
  double spread = 0.0;  // the track's spread (MinorSpread of the inliers), metres
  double sigma = 0.0;   // median sigma of the inliers
  int inliers = 0;
  FixClass cls = FixClass::kNone;  // the worst class among the inliers
  std::vector<char> inlier;
};

// The track's spread: its extent across its own main direction (the extent along the minor
// principal axis of the points). A straight drive has a spread of about 0 however long it is, so
// it stays `aligning` (§11.2: "a straight line stays aligning").
double MinorSpread(const std::vector<double> & xs, const std::vector<double> & ys);

// Weighted closed-form rigid fit, scale fixed to 1 (w = 1/sigma^2), with outliers rejected: a pair
// whose residual is over k * max(1.4826 * median residual, its own sigma) is dropped and the fit
// repeated, up to `iterations` times. The scale is fitted once more, free, as the diagnostic.
RigidFit FitPairs(const std::vector<Pair> & pairs, double outlier_k = 3.0, int iterations = 5);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_GEO_HPP_
