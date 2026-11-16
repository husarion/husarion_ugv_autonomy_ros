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

// §11.2: a straight line stays `aligning`; the hysteresis; need_m on RTK and on plain fixes;
// `held` to `poor` when returning fixes disagree; and the rest of §5.4's word table.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "husarion_ugv_map_gateway/align.hpp"

using namespace husarion_ugv_map_gateway;

namespace
{

const GeoRef kTruth{51.1079, 17.0385, 0.6};

// The robot drives `path(t)` in the map frame; the receiver reports the truth plus noise, once a
// second. `offset_m` shifts every fix east (a fix that disagrees).
class Drive
{
public:
  Drive(Aligner & aligner, double sigma, int status, double cov_scale = 1.0)
  : aligner_(aligner), sigma_(sigma), status_(status), noise_(0.0, sigma * cov_scale)
  {
  }

  void Run(
    double from, double to, const std::function<std::pair<double, double>(double)> & path,
    double offset_m = 0.0, bool with_fixes = true)
  {
    for (double t = from; t < to - 1e-9; t += 1.0) {
      if (with_fixes) {
        const auto [x, y] = path(t);
        LatLon ll = MapToLatLon(kTruth, x, y);
        if (offset_m != 0.0) {
          ll = EnuToLatLon(ll, {offset_m, 0.0});
        }
        ll = EnuToLatLon(ll, {noise_(rng_), noise_(rng_)});
        FixInput in;
        in.t = t;
        in.ll = ll;
        in.quality = ClassifyFix(status_, 2, sigma_ * sigma_, sigma_ * sigma_, FixClassParams());
        in.has_map = true;
        in.x = x;
        in.y = y;
        aligner_.AddFix(in);
      }
      aligner_.Tick(t);
    }
  }

private:
  Aligner & aligner_;
  double sigma_;
  int status_;
  std::mt19937 rng_{11};
  std::normal_distribution<double> noise_;
};

// 1 m/s: 30 m east, then north, then west, then south (a 30 m square), and around again.
std::pair<double, double> Square(double t)
{
  const double s = std::fmod(t, 120.0);
  if (s < 30) {
    return {s, 0.0};
  }
  if (s < 60) {
    return {30.0, s - 30};
  }
  if (s < 90) {
    return {90 - s, 30.0};
  }
  return {0.0, 120 - s};
}

std::pair<double, double> Line(double t) { return {t * std::cos(0.2), t * std::sin(0.2)}; }

}  // namespace

TEST(Align, StartsNotAligned)
{
  Aligner a;
  EXPECT_EQ(a.word(), Word::kNotAligned);
  EXPECT_EQ(a.version(), 0u);
  EXPECT_FALSE(a.alignment().has_value());
  Drive d(a, 0.02, 2);
  d.Run(0, 9, Square);  // 9 pairs: fewer than 10
  EXPECT_EQ(a.word(), Word::kNotAligned);
  EXPECT_DOUBLE_EQ(a.need_m(), 0.0);
}

TEST(Align, AStraightLineStaysAligning)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 200, Line);  // 200 m dead straight with RTK
  EXPECT_EQ(a.word(), Word::kAligning);
  EXPECT_FALSE(a.alignment().has_value());
  EXPECT_NEAR(a.need_m(), 10.0, 0.2);
}

TEST(Align, NeedOnRtkAndOnPlain)
{
  // RTK (sigma 2 cm): the spread needed is 10 m.
  Aligner rtk;
  Drive r(rtk, 0.02, 2);
  r.Run(0, 36, Square);  // 30 m east, 6 m north: spread ~ 6 m
  EXPECT_EQ(rtk.word(), Word::kAligning);
  EXPECT_DOUBLE_EQ(rtk.required_spread(), 10.0);
  EXPECT_NEAR(rtk.need_m(), 10.0 - rtk.fit().spread, 1e-9);
  EXPECT_GT(rtk.need_m(), 2.0);

  // Plain (sigma 2 m): 20 x 2 = 40 m, not 10.
  Aligner plain;
  Drive p(plain, 2.0, 0);
  p.Run(0, 60, Square);  // 30 m east, 30 m north
  EXPECT_EQ(plain.word(), Word::kAligning);
  EXPECT_NEAR(plain.required_spread(), 40.0, 4.0);
  EXPECT_NEAR(plain.need_m(), plain.required_spread() - plain.fit().spread, 1e-9);
  EXPECT_GT(plain.need_m(), 5.0);
}

TEST(Align, BecomesAlignedOnlyAfterTenGoodSeconds)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  // The spread reaches 10 m at t = 40 (30 m east, then 10 m north); one tick later the fit is good.
  d.Run(0, 41, Square);
  EXPECT_EQ(a.word(), Word::kAligning);
  d.Run(41, 50, Square);
  EXPECT_EQ(a.word(), Word::kAligning) << "better words wait 10 s";
  d.Run(50, 53, Square);
  EXPECT_EQ(a.word(), Word::kAligned);
  ASSERT_TRUE(a.alignment().has_value());
  EXPECT_EQ(a.version(), 1u);
  EXPECT_EQ(a.alignment()->source, Source::kFit);
  EXPECT_EQ(a.alignment()->fix, FixClass::kRtkFixed);
  EXPECT_NEAR(a.alignment()->geo.lat0, kTruth.lat0, 1e-6);
  EXPECT_NEAR(a.alignment()->geo.lon0, kTruth.lon0, 1e-6);
  EXPECT_NEAR(a.alignment()->geo.yaw, kTruth.yaw, 0.01);
  EXPECT_DOUBLE_EQ(a.err_m(), 0.2);  // the rtk_fixed floor, not the 2 cm residual
  EXPECT_DOUBLE_EQ(a.need_m(), 0.0);
}

TEST(Align, RefinementsDoNotMoveTheVersion)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 60, Square);
  ASSERT_EQ(a.word(), Word::kAligned);
  const uint32_t v = a.version();
  d.Run(60, 300, Square);
  EXPECT_EQ(a.word(), Word::kAligned);
  EXPECT_EQ(a.version(), v);
  EXPECT_GT(a.alignment()->pairs, 100);
}

TEST(Align, HeldWhenFixesStopAndBackWhenTheyAgree)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 60, Square);
  ASSERT_EQ(a.word(), Word::kAligned);
  const Alignment before = *a.alignment();
  const uint32_t v = a.version();
  d.Run(60, 69, Square, 0.0, false);
  EXPECT_EQ(a.word(), Word::kAligned);
  d.Run(69, 71, Square, 0.0, false);
  EXPECT_EQ(a.word(), Word::kHeld);  // 10 s without a usable fix
  d.Run(71, 200, Square, 0.0, false);
  EXPECT_EQ(a.word(), Word::kHeld);
  EXPECT_DOUBLE_EQ(a.alignment()->geo.yaw, before.geo.yaw);  // held numbers do not change
  EXPECT_EQ(a.version(), v);
  // Fixes return and agree: aligned after 10 s of them.
  d.Run(200, 209, Square);
  EXPECT_EQ(a.word(), Word::kHeld);
  d.Run(209, 212, Square);
  EXPECT_EQ(a.word(), Word::kAligned);
}

TEST(Align, HeldToPoorWhenReturningFixesDisagree)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 60, Square);
  d.Run(60, 80, Square, 0.0, false);
  ASSERT_EQ(a.word(), Word::kHeld);
  const Alignment held = *a.alignment();
  // The fixes come back 5 m off (more than 3 x err_m = 0.6 m).
  d.Run(80, 109, Square, 5.0);
  EXPECT_EQ(a.word(), Word::kHeld);
  d.Run(109, 112, Square, 5.0);
  EXPECT_EQ(a.word(), Word::kPoor);
  EXPECT_DOUBLE_EQ(a.alignment()->geo.lat0, held.geo.lat0);  // the numbers stay
}

TEST(Align, SidecarLoadsAsHeldAndDisagreementGivesPoor)
{
  Alignment saved;
  saved.geo = kTruth;
  saved.rms_m = 1.0;
  saved.fix = FixClass::kPlain;
  saved.pairs = 50;
  Aligner a;
  a.Load(saved);
  EXPECT_EQ(a.word(), Word::kHeld);
  EXPECT_EQ(a.alignment()->source, Source::kSidecar);
  EXPECT_EQ(a.version(), 1u);
  Drive d(a, 0.5, 0);
  // err_m is the plain floor, 2 m; fixes 10 m off disagree.
  d.Run(0, 31, Square, 10.0);
  EXPECT_EQ(a.word(), Word::kPoor);
}

TEST(Align, SidecarLoadedThenAgreeingFixesGiveAligned)
{
  Alignment saved;
  saved.geo = kTruth;
  saved.rms_m = 0.5;
  saved.fix = FixClass::kPlain;
  Aligner a;
  a.Load(saved);
  Drive d(a, 0.3, 0);
  d.Run(0, 12, Square);
  EXPECT_EQ(a.word(), Word::kAligned);
  EXPECT_EQ(a.alignment()->source, Source::kSidecar);
}

TEST(Align, OnlyAlignedBecomesHeld)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 30, Line);
  ASSERT_EQ(a.word(), Word::kAligning);
  d.Run(30, 100, Line, 0.0, false);
  EXPECT_EQ(a.word(), Word::kAligning);  // aligning keeps its pairs and waits
}

TEST(Align, LargeResidualIsPoor)
{
  // Fixes with 6 m noise but reported as 2 m: the fit's rms is over 3 m.
  Aligner a;
  Drive d(a, 2.0, 0, 3.0);
  d.Run(0, 400, Square);
  EXPECT_EQ(a.word(), Word::kPoor);
  ASSERT_TRUE(a.alignment().has_value());
  EXPECT_GT(a.alignment()->rms_m, 3.0);
}

TEST(Align, WorseWordsWaitFiveSeconds)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 60, Square);
  ASSERT_EQ(a.word(), Word::kAligned);
  // Fixes 5 m off: the fit turns bad and they disagree; a worse word waits 5 s either way.
  d.Run(60, 64, Square, 5.0);
  EXPECT_EQ(a.word(), Word::kAligned);
  d.Run(64, 97, Square, 5.0);
  EXPECT_EQ(a.word(), Word::kPoor);
}

TEST(Align, ResetBumpsTheVersionAndClears)
{
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 60, Square);
  ASSERT_EQ(a.version(), 1u);
  a.Reset();
  EXPECT_EQ(a.word(), Word::kNotAligned);
  EXPECT_FALSE(a.alignment().has_value());
  EXPECT_EQ(a.version(), 2u);  // "has an alignment" to "has none"
  EXPECT_EQ(a.pair_count(), 0u);
}

TEST(Align, AMovedAlignmentBumpsTheVersion)
{
  Alignment saved;
  saved.geo = kTruth;
  saved.rms_m = 0.05;
  saved.fix = FixClass::kRtkFixed;
  Aligner a;
  a.SetMapBounds(0, 0, 30, 30);
  a.Load(saved);
  ASSERT_EQ(a.version(), 1u);
  // The truth is 1 degree off the sidecar's yaw: 0.5 m at the map's far corner (42 m out).
  Aligner b;
  b.SetMapBounds(0, 0, 30, 30);
  Alignment off = saved;
  off.geo.yaw += M_PI / 180.0;
  b.Load(off);
  Drive d(b, 0.02, 2);
  d.Run(0, 200, Square);
  EXPECT_EQ(b.word(), Word::kAligned);
  EXPECT_EQ(b.version(), 2u);  // the new fit moved the far edge by more than 0.2 m
  EXPECT_EQ(b.alignment()->source, Source::kFit);
}

TEST(Align, RestoreFromTheSession)
{
  Alignment s;
  s.geo = kTruth;
  Aligner a;
  a.Restore(Word::kAligned, s, 7);
  EXPECT_EQ(a.word(), Word::kHeld);
  EXPECT_EQ(a.version(), 7u);
  Aligner b;
  b.Restore(Word::kAligning, std::nullopt, 3);
  EXPECT_EQ(b.word(), Word::kNotAligned);
  EXPECT_EQ(b.version(), 3u);
}

TEST(Align, Words)
{
  for (Word w : {Word::kNotAligned, Word::kAligning, Word::kAligned, Word::kPoor, Word::kHeld}) {
    EXPECT_EQ(ParseWord(WordName(w)), w);
  }
  EXPECT_TRUE(WordServesLL(Word::kAligned));
  EXPECT_TRUE(WordServesLL(Word::kHeld));
  EXPECT_FALSE(WordServesLL(Word::kPoor));
  EXPECT_FALSE(WordServesLL(Word::kAligning));
  EXPECT_FALSE(WordServesLL(Word::kNotAligned));
  EXPECT_TRUE(WordIsOk(Word::kHeld));
  EXPECT_FALSE(WordIsOk(Word::kPoor));
}

TEST(Align, ANewFitReplacesASidecarTheFixesDisagreeWith)
{
  // The sidecar is 1 m off the truth (over 3 x the 0.2 m RTK floor): held, then poor once the
  // fixes have disagreed for 30 s, then aligned on a fit of their own, which moves the version.
  Alignment saved;
  const LatLon shifted = EnuToLatLon({kTruth.lat0, kTruth.lon0}, {0.0, 1.0});
  saved.geo = {shifted.lat, shifted.lon, kTruth.yaw};
  saved.rms_m = 0.05;
  saved.fix = FixClass::kRtkFixed;
  Aligner a;
  a.SetMapBounds(0, 0, 30, 30);
  a.Load(saved);
  Drive d(a, 0.02, 2);
  d.Run(0, 29, Square);
  EXPECT_EQ(a.word(), Word::kHeld);
  d.Run(29, 32, Square);
  EXPECT_EQ(a.word(), Word::kPoor);
  d.Run(32, 80, Square);
  EXPECT_EQ(a.word(), Word::kAligned);
  EXPECT_EQ(a.alignment()->source, Source::kFit);
  EXPECT_EQ(a.version(), 2u);
  EXPECT_NEAR(a.alignment()->geo.lat0, kTruth.lat0, 1e-7);
}

TEST(Align, AGoodFitDoesNotHideFixesThatJumped)
{
  // Aligned on a long track; then every fix jumps 5 m. The fit of all pairs still looks good
  // (the new ones are its outliers), but the newest fixes disagree with it: poor.
  Aligner a;
  Drive d(a, 0.02, 2);
  d.Run(0, 300, Square);
  ASSERT_EQ(a.word(), Word::kAligned);
  d.Run(300, 340, Square, 5.0);
  EXPECT_EQ(a.word(), Word::kPoor);
}
