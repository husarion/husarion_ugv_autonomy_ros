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

#include "husarion_ugv_map_gateway/geo.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace husarion_ugv_map_gateway
{

namespace
{

constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDeg = M_PI / 180.0;

struct Ecef
{
  double x, y, z;
};

Ecef ToEcef(const LatLon & p, double h = 0.0)
{
  const double phi = p.lat * kDeg, lam = p.lon * kDeg;
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
  return {
    (n + h) * std::cos(phi) * std::cos(lam), (n + h) * std::cos(phi) * std::sin(lam),
    (n * (1.0 - kE2) + h) * std::sin(phi)};
}

LatLon FromEcef(const Ecef & q)
{
  const double lon = std::atan2(q.y, q.x);
  const double p = std::hypot(q.x, q.y);
  double phi = std::atan2(q.z, p * (1.0 - kE2));
  for (int i = 0; i < 8; ++i) {
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(phi) * std::sin(phi));
    const double h = p / std::cos(phi) - n;
    phi = std::atan2(q.z, p * (1.0 - kE2 * n / (n + h)));
  }
  return {phi / kDeg, lon / kDeg};
}

void Enu(const LatLon & ref, const LatLon & p, double & e, double & n, double & u)
{
  const Ecef o = ToEcef(ref), q = ToEcef(p);
  const double dx = q.x - o.x, dy = q.y - o.y, dz = q.z - o.z;
  const double sp = std::sin(ref.lat * kDeg), cp = std::cos(ref.lat * kDeg);
  const double sl = std::sin(ref.lon * kDeg), cl = std::cos(ref.lon * kDeg);
  e = -sl * dx + cl * dy;
  n = -sp * cl * dx - sp * sl * dy + cp * dz;
  u = cp * cl * dx + cp * sl * dy + sp * dz;
}

LatLon FromEnu(const LatLon & ref, double e, double n, double u)
{
  const Ecef o = ToEcef(ref);
  const double sp = std::sin(ref.lat * kDeg), cp = std::cos(ref.lat * kDeg);
  const double sl = std::sin(ref.lon * kDeg), cl = std::cos(ref.lon * kDeg);
  return FromEcef(
    {o.x - sl * e - sp * cl * n + cp * cl * u, o.y + cl * e - sp * sl * n + cp * sl * u,
     o.z + cp * n + sp * u});
}

bool ValidLatLon(const LatLon & p)
{
  return std::isfinite(p.lat) && std::isfinite(p.lon) && std::abs(p.lat) <= 90.0 &&
         std::abs(p.lon) <= 180.0;
}

double Median(std::vector<double> v)
{
  if (v.empty()) {
    return 0.0;
  }
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

}  // namespace

EastNorth LatLonToEnu(const LatLon & ref, const LatLon & p)
{
  double e, n, u;
  Enu(ref, p, e, n, u);
  return {e, n};
}

LatLon EnuToLatLon(const LatLon & ref, const EastNorth & en)
{
  // The ellipsoid point whose projection on ref's tangent plane is (e, n): start on the plane
  // and move along ref's up until the height is 0, so LatLonToEnu inverts it exactly.
  double u = 0.0;
  LatLon p = FromEnu(ref, en.e, en.n, u);
  for (int i = 0; i < 3; ++i) {
    double e, n, up;
    Enu(ref, p, e, n, up);
    u = up;
    p = FromEnu(ref, en.e, en.n, u);
  }
  return p;
}

LatLon MapToLatLon(const GeoRef & ref, double x, double y)
{
  const double c = std::cos(ref.yaw), s = std::sin(ref.yaw);
  return EnuToLatLon({ref.lat0, ref.lon0}, {c * x - s * y, s * x + c * y});
}

std::optional<EastNorth> LatLonToMap(const GeoRef & ref, const LatLon & p)
{
  if (!ValidLatLon(p)) {
    return std::nullopt;
  }
  const EastNorth en = LatLonToEnu({ref.lat0, ref.lon0}, p);
  const double c = std::cos(ref.yaw), s = std::sin(ref.yaw);
  return EastNorth{c * en.e + s * en.n, -s * en.e + c * en.n};
}

const char * FixClassName(FixClass cls)
{
  switch (cls) {
    case FixClass::kRtkFixed:
      return "rtk_fixed";
    case FixClass::kRtkFloat:
      return "rtk_float";
    case FixClass::kPlain:
      return "plain";
    default:
      return "none";
  }
}

std::optional<FixClass> ParseFixClass(const std::string & name)
{
  for (FixClass c : {FixClass::kNone, FixClass::kPlain, FixClass::kRtkFloat, FixClass::kRtkFixed}) {
    if (name == FixClassName(c)) {
      return c;
    }
  }
  return std::nullopt;
}

double ErrFloor(FixClass cls, const ErrFloors & floors)
{
  switch (cls) {
    case FixClass::kRtkFixed:
      return floors.rtk_fixed;
    case FixClass::kRtkFloat:
      return floors.rtk_float;
    default:
      return floors.plain;
  }
}

FixQuality ClassifyFix(
  int status, uint8_t covariance_type, double var_east, double var_north,
  const FixClassParams & params)
{
  constexpr int kStatusGbas = 2;
  const bool rtk = status == kStatusGbas;
  const double var = std::max(var_east, var_north);
  double sigma;
  if (covariance_type != 0 && std::isfinite(var) && var > 0.0) {
    sigma = std::sqrt(var);
  } else {
    sigma = rtk ? params.default_rtk_sigma : params.default_plain_sigma;
  }
  FixQuality q;
  q.sigma = sigma;
  if (rtk && sigma <= params.rtk_fixed_max_sigma) {
    q.cls = FixClass::kRtkFixed;
  } else if (rtk && sigma <= params.rtk_float_max_sigma) {
    q.cls = FixClass::kRtkFloat;
  } else {
    q.cls = FixClass::kPlain;
  }
  return q;
}

bool UsableFix(int status, double lat, double lon)
{
  return status >= 0 && ValidLatLon({lat, lon}) && !(lat == 0.0 && lon == 0.0);
}

double MinorSpread(const std::vector<double> & xs, const std::vector<double> & ys)
{
  const size_t n = std::min(xs.size(), ys.size());
  if (n < 2) {
    return 0.0;
  }
  double mx = 0, my = 0;
  for (size_t i = 0; i < n; ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;
  double sxx = 0, syy = 0, sxy = 0;
  for (size_t i = 0; i < n; ++i) {
    sxx += (xs[i] - mx) * (xs[i] - mx);
    syy += (ys[i] - my) * (ys[i] - my);
    sxy += (xs[i] - mx) * (ys[i] - my);
  }
  // The major axis is at 0.5 atan2(2 sxy, sxx - syy); the minor one is perpendicular to it.
  const double major = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const double ux = -std::sin(major), uy = std::cos(major);
  double lo = 1e300, hi = -1e300;
  for (size_t i = 0; i < n; ++i) {
    const double d = (xs[i] - mx) * ux + (ys[i] - my) * uy;
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  return hi - lo;
}

namespace
{

RigidFit SolveRigid(const std::vector<Pair> & pairs, const std::vector<char> & use)
{
  RigidFit fit;
  double sw = 0, px = 0, py = 0, qx = 0, qy = 0;
  for (size_t i = 0; i < pairs.size(); ++i) {
    if (!use[i]) {
      continue;
    }
    const double w = 1.0 / std::max(pairs[i].sigma * pairs[i].sigma, 1e-6);
    sw += w;
    px += w * pairs[i].x;
    py += w * pairs[i].y;
    qx += w * pairs[i].e;
    qy += w * pairs[i].n;
  }
  if (sw <= 0.0) {
    return fit;
  }
  px /= sw;
  py /= sw;
  qx /= sw;
  qy /= sw;
  double a = 0, b = 0, pp = 0;
  for (size_t i = 0; i < pairs.size(); ++i) {
    if (!use[i]) {
      continue;
    }
    const double w = 1.0 / std::max(pairs[i].sigma * pairs[i].sigma, 1e-6);
    const double dx = pairs[i].x - px, dy = pairs[i].y - py;
    const double ex = pairs[i].e - qx, ey = pairs[i].n - qy;
    a += w * (dx * ex + dy * ey);
    b += w * (dx * ey - dy * ex);
    pp += w * (dx * dx + dy * dy);
  }
  if (pp <= 1e-9 || (a == 0.0 && b == 0.0)) {
    return fit;
  }
  fit.yaw = std::atan2(b, a);
  const double c = std::cos(fit.yaw), s = std::sin(fit.yaw);
  fit.tx = qx - (c * px - s * py);
  fit.ty = qy - (s * px + c * py);
  fit.scale = std::hypot(a, b) / pp;
  fit.valid = true;
  return fit;
}

double Residual(const RigidFit & fit, const Pair & p)
{
  const double c = std::cos(fit.yaw), s = std::sin(fit.yaw);
  return std::hypot(c * p.x - s * p.y + fit.tx - p.e, s * p.x + c * p.y + fit.ty - p.n);
}

}  // namespace

RigidFit FitPairs(const std::vector<Pair> & pairs, double outlier_k, int iterations)
{
  std::vector<char> use(pairs.size(), 1);
  RigidFit fit;
  for (int it = 0; it <= iterations; ++it) {
    fit = SolveRigid(pairs, use);
    if (!fit.valid) {
      return fit;
    }
    std::vector<double> residuals;
    for (size_t i = 0; i < pairs.size(); ++i) {
      if (use[i]) {
        residuals.push_back(Residual(fit, pairs[i]));
      }
    }
    const double robust = 1.4826 * Median(residuals);
    bool changed = false;
    for (size_t i = 0; i < pairs.size(); ++i) {
      const bool keep = Residual(fit, pairs[i]) <= outlier_k * std::max(robust, pairs[i].sigma);
      if (bool(use[i]) != keep) {
        use[i] = keep;
        changed = true;
      }
    }
    if (std::count(use.begin(), use.end(), 1) < 2) {
      fit.valid = false;
      return fit;
    }
    if (!changed) {
      break;
    }
  }
  fit = SolveRigid(pairs, use);
  if (!fit.valid) {
    return fit;
  }
  fit.inlier = use;
  double sum2 = 0;
  std::vector<double> xs, ys, sigmas;
  fit.cls = FixClass::kRtkFixed;
  for (size_t i = 0; i < pairs.size(); ++i) {
    if (!use[i]) {
      continue;
    }
    const double r = Residual(fit, pairs[i]);
    sum2 += r * r;
    xs.push_back(pairs[i].x);
    ys.push_back(pairs[i].y);
    sigmas.push_back(pairs[i].sigma);
    fit.cls = std::min(fit.cls, pairs[i].cls);
    ++fit.inliers;
  }
  fit.rms = std::sqrt(sum2 / fit.inliers);
  fit.spread = MinorSpread(xs, ys);
  fit.sigma = Median(sigmas);
  return fit;
}

}  // namespace husarion_ugv_map_gateway
