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

// §11.2: the fit recovers a known yaw and translation from a synthetic track with noise; outliers
// are rejected; the scale diagnostic; fromLL and toLL invert each other.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "husarion_ugv_map_gateway/geo.hpp"

using namespace husarion_ugv_map_gateway;

namespace
{

const LatLon kWroclaw{51.1079, 17.0385};

// An L-shaped drive: 60 m east-ish, then 40 m north-ish in the map frame, one point per metre.
std::vector<std::pair<double, double>> LTrack()
{
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i <= 60; ++i) {
    pts.emplace_back(i, 0.0);
  }
  for (int i = 1; i <= 40; ++i) {
    pts.emplace_back(60.0, i);
  }
  return pts;
}

std::vector<Pair> MakePairs(
  const std::vector<std::pair<double, double>> & track, double yaw, double tx, double ty,
  double sigma, unsigned seed, double scale = 1.0)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, sigma);
  std::vector<Pair> pairs;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const auto & [x, y] : track) {
    Pair p;
    p.x = x;
    p.y = y;
    p.e = scale * (c * x - s * y) + tx + noise(rng);
    p.n = scale * (s * x + c * y) + ty + noise(rng);
    p.sigma = sigma;
    p.cls = FixClass::kPlain;
    pairs.push_back(p);
  }
  return pairs;
}

double AngleDiff(double a, double b) { return std::atan2(std::sin(a - b), std::cos(a - b)); }

}  // namespace

TEST(Geo, TangentPlaneRoundTrip)
{
  for (double e : {-5000.0, -12.5, 0.0, 731.0, 5000.0}) {
    for (double n : {-5000.0, 0.0, 44.4, 5000.0}) {
      const LatLon p = EnuToLatLon(kWroclaw, {e, n});
      const EastNorth back = LatLonToEnu(kWroclaw, p);
      EXPECT_NEAR(back.e, e, 1e-6);
      EXPECT_NEAR(back.n, n, 1e-6);
    }
  }
  // One degree of latitude at 51 N is about 111.25 km.
  EXPECT_NEAR(LatLonToEnu(kWroclaw, {kWroclaw.lat + 1.0, kWroclaw.lon}).n, 111250.0, 300.0);
}

TEST(Geo, FromLLAndToLLInvertEachOther)
{
  const GeoRef ref{50.0, 19.9, 0.7};
  for (double x : {-3000.0, -1.0, 0.0, 250.0, 3000.0}) {
    for (double y : {-3000.0, 0.0, 17.0, 3000.0}) {
      const LatLon ll = MapToLatLon(ref, x, y);
      const auto back = LatLonToMap(ref, ll);
      ASSERT_TRUE(back.has_value());
      EXPECT_NEAR(back->e, x, 1e-6);
      EXPECT_NEAR(back->n, y, 1e-6);
    }
  }
  // map x points `yaw` counter-clockwise from east: with yaw = pi/2, map x is north.
  const GeoRef north{50.0, 19.9, M_PI / 2};
  const EastNorth en = LatLonToEnu({50.0, 19.9}, MapToLatLon(north, 100.0, 0.0));
  EXPECT_NEAR(en.e, 0.0, 1e-6);
  EXPECT_NEAR(en.n, 100.0, 1e-6);
}

TEST(Geo, LatLonToMapRefusesNonsense)
{
  const GeoRef ref{50.0, 19.9, 0.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(LatLonToMap(ref, {nan, 19.9}).has_value());
  EXPECT_FALSE(LatLonToMap(ref, {91.0, 19.9}).has_value());
  EXPECT_FALSE(LatLonToMap(ref, {50.0, 181.0}).has_value());
}

TEST(Fit, RecoversYawAndTranslationWithPlainNoise)
{
  const double yaw = 0.83, tx = -12.0, ty = 35.0;
  const RigidFit fit = FitPairs(MakePairs(LTrack(), yaw, tx, ty, 2.0, 1));
  ASSERT_TRUE(fit.valid);
  EXPECT_LT(std::abs(AngleDiff(fit.yaw, yaw)), 2.0 * M_PI / 180.0);
  EXPECT_NEAR(fit.tx, tx, 1.5);
  EXPECT_NEAR(fit.ty, ty, 1.5);
  EXPECT_NEAR(fit.rms, 2.0 * std::sqrt(2.0), 0.6);  // 2-D residual of a 2 m sigma per axis
  EXPECT_NEAR(fit.scale, 1.0, 0.02);
  EXPECT_EQ(fit.inliers, 101);
}

TEST(Fit, RecoversYawAndTranslationWithRtkNoise)
{
  const double yaw = -2.1, tx = 3.0, ty = -4.0;
  const RigidFit fit = FitPairs(MakePairs(LTrack(), yaw, tx, ty, 0.02, 2));
  ASSERT_TRUE(fit.valid);
  EXPECT_LT(std::abs(AngleDiff(fit.yaw, yaw)), 0.05 * M_PI / 180.0);
  EXPECT_NEAR(fit.tx, tx, 0.03);
  EXPECT_NEAR(fit.ty, ty, 0.03);
  EXPECT_LT(fit.rms, 0.05);
}

TEST(Fit, RejectsOutliers)
{
  const double yaw = 0.3, tx = 5.0, ty = 5.0;
  auto pairs = MakePairs(LTrack(), yaw, tx, ty, 0.5, 3);
  // Multipath: every tenth fix 30 to 60 m off.
  for (size_t i = 0; i < pairs.size(); i += 10) {
    pairs[i].e += 30.0 + double(i % 30);
    pairs[i].n -= 20.0;
  }
  const RigidFit fit = FitPairs(pairs);
  ASSERT_TRUE(fit.valid);
  EXPECT_EQ(fit.inliers, int(pairs.size()) - 11);
  EXPECT_LT(std::abs(AngleDiff(fit.yaw, yaw)), 0.5 * M_PI / 180.0);
  EXPECT_NEAR(fit.tx, tx, 0.3);
  EXPECT_NEAR(fit.ty, ty, 0.3);
  EXPECT_LT(fit.rms, 1.0);
  for (size_t i = 0; i < pairs.size(); i += 10) {
    EXPECT_FALSE(fit.inlier[i]) << i;
  }
}

TEST(Fit, ScaleDiagnostic)
{
  // Odometry 5 % long: the rigid fit still runs, the diagnostic says 1.05.
  const RigidFit fit = FitPairs(MakePairs(LTrack(), 0.1, 0.0, 0.0, 0.02, 4, 1.05));
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.scale, 1.05, 0.002);
  const RigidFit ok = FitPairs(MakePairs(LTrack(), 0.1, 0.0, 0.0, 0.02, 4, 1.0));
  EXPECT_NEAR(ok.scale, 1.0, 0.002);
}

TEST(Fit, SpreadOfAStraightLineIsAboutZero)
{
  std::vector<double> xs, ys;
  for (int i = 0; i < 200; ++i) {
    xs.push_back(i * 0.5 * std::cos(0.4));
    ys.push_back(i * 0.5 * std::sin(0.4));
  }
  EXPECT_LT(MinorSpread(xs, ys), 1e-9);
  // A 20 m square loop spreads 20 m across.
  xs.clear();
  ys.clear();
  for (int i = 0; i < 20; ++i) {
    xs.insert(xs.end(), {double(i), 20.0, double(20 - i), 0.0});
    ys.insert(ys.end(), {0.0, double(i), 20.0, double(20 - i)});
  }
  EXPECT_NEAR(MinorSpread(xs, ys), 20.0, 0.01);
}

TEST(Fix, Classes)
{
  const FixClassParams p;
  // RTK needs status GBAS (2).
  EXPECT_EQ(ClassifyFix(2, 2, 0.0004, 0.0004, p).cls, FixClass::kRtkFixed);
  EXPECT_EQ(ClassifyFix(2, 2, 0.09, 0.04, p).cls, FixClass::kRtkFloat);
  EXPECT_NEAR(ClassifyFix(2, 2, 0.09, 0.04, p).sigma, 0.3, 1e-9);
  EXPECT_EQ(ClassifyFix(2, 2, 4.0, 4.0, p).cls, FixClass::kPlain);
  EXPECT_EQ(ClassifyFix(0, 2, 0.0004, 0.0004, p).cls, FixClass::kPlain);
  // No covariance: the defaults, never better than rtk_float.
  EXPECT_EQ(ClassifyFix(2, 0, 0.0, 0.0, p).cls, FixClass::kRtkFloat);
  EXPECT_EQ(ClassifyFix(0, 0, 0.0, 0.0, p).cls, FixClass::kPlain);
  EXPECT_DOUBLE_EQ(ClassifyFix(0, 0, 0.0, 0.0, p).sigma, 2.5);
  EXPECT_DOUBLE_EQ(ClassifyFix(0, 1, std::numeric_limits<double>::infinity(), 1.0, p).sigma, 2.5);
}

TEST(Fix, UsableFixesOnly)
{
  EXPECT_TRUE(UsableFix(0, 51.1, 17.0));
  EXPECT_FALSE(UsableFix(-1, 51.1, 17.0));
  EXPECT_FALSE(UsableFix(-2, 51.1, 17.0));
  EXPECT_FALSE(UsableFix(0, 0.0, 0.0));
  EXPECT_FALSE(UsableFix(0, std::numeric_limits<double>::quiet_NaN(), 17.0));
  EXPECT_FALSE(UsableFix(0, 51.1, std::numeric_limits<double>::infinity()));
}

TEST(Fix, ErrFloors)
{
  const ErrFloors f;
  EXPECT_DOUBLE_EQ(ErrFloor(FixClass::kPlain, f), 2.0);
  EXPECT_DOUBLE_EQ(ErrFloor(FixClass::kRtkFloat, f), 0.5);
  EXPECT_DOUBLE_EQ(ErrFloor(FixClass::kRtkFixed, f), 0.2);
}
