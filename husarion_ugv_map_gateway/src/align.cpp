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

#include "husarion_ugv_map_gateway/align.hpp"

#include <algorithm>
#include <cmath>

namespace husarion_ugv_map_gateway
{

const char * WordName(Word word)
{
  switch (word) {
    case Word::kAligning:
      return "aligning";
    case Word::kAligned:
      return "aligned";
    case Word::kPoor:
      return "poor";
    case Word::kHeld:
      return "held";
    default:
      return "not_aligned";
  }
}

std::optional<Word> ParseWord(const std::string & name)
{
  for (Word w : {Word::kNotAligned, Word::kAligning, Word::kAligned, Word::kPoor, Word::kHeld}) {
    if (name == WordName(w)) {
      return w;
    }
  }
  return std::nullopt;
}

bool WordIsOk(Word word) { return word == Word::kAligned || word == Word::kHeld; }

const char * SourceName(Source source) { return source == Source::kSidecar ? "sidecar" : "fit"; }

Aligner::Aligner(const AlignParams & params) : params_(params) {}

int Aligner::Rank(Word word)
{
  switch (word) {
    case Word::kAligned:
      return 4;
    case Word::kHeld:
      return 3;
    case Word::kAligning:
    case Word::kPoor:
      return 1;
    default:
      return 0;
  }
}

void Aligner::Reset()
{
  if (alignment_) {
    ++version_;
  }
  alignment_.reset();
  version_ref_.reset();
  word_ = Word::kNotAligned;
  origin_.reset();
  pairs_.clear();
  fit_ = RigidFit();
  ++fit_serial_;
  applied_serial_ = fit_serial_;
  agree_since_.reset();
  disagree_since_.reset();
  candidate_.reset();
}

void Aligner::Load(const Alignment & alignment)
{
  Reset();
  Alignment a = alignment;
  a.source = Source::kSidecar;
  SetNumbers(a);
  word_ = Word::kHeld;
}

void Aligner::Restore(Word word, const std::optional<Alignment> & alignment, uint32_t version)
{
  Reset();
  version_ = version;
  if (!alignment) {
    return;
  }
  if (word == Word::kAligned || word == Word::kHeld) {
    word_ = Word::kHeld;
  } else if (word == Word::kPoor) {
    word_ = Word::kPoor;
  } else {
    return;
  }
  alignment_ = alignment;
  version_ref_ = alignment;
}

void Aligner::SetMapBounds(double min_x, double min_y, double max_x, double max_y)
{
  has_bounds_ = true;
  bounds_[0] = min_x;
  bounds_[1] = min_y;
  bounds_[2] = max_x;
  bounds_[3] = max_y;
}

void Aligner::AddFix(const FixInput & fix)
{
  if (last_fix_t_ && fix.t - *last_fix_t_ > params_.fix_gap_s) {
    agree_since_.reset();
    disagree_since_.reset();
  }
  last_fix_t_ = fix.t;
  last_fix_class_ = fix.quality.cls;
  if (!fix.has_map) {
    return;
  }

  if (alignment_) {
    const auto at = LatLonToMap(alignment_->geo, fix.ll);
    const double off = at ? std::hypot(at->e - fix.x, at->n - fix.y) : 1e300;
    if (off > params_.disagree_factor * err_m()) {
      agree_since_.reset();
      if (!disagree_since_) {
        disagree_since_ = fix.t;
      }
    } else {
      disagree_since_.reset();
      if (!agree_since_) {
        agree_since_ = fix.t;
      }
    }
  }

  if (!origin_) {
    origin_ = fix.ll;
  }
  if (!pairs_.empty()) {
    const Pair & last = pairs_.back();
    if (std::hypot(fix.x - last.x, fix.y - last.y) < params_.pair_min_distance_m) {
      return;
    }
  }
  const EastNorth en = LatLonToEnu(*origin_, fix.ll);
  Pair p;
  p.x = fix.x;
  p.y = fix.y;
  p.e = en.e;
  p.n = en.n;
  p.sigma = fix.quality.sigma;
  p.cls = fix.quality.cls;
  pairs_.push_back(p);
  if (pairs_.size() > params_.max_pairs) {
    pairs_.erase(pairs_.begin(), pairs_.begin() + (pairs_.size() - params_.max_pairs));
  }
  if (static_cast<int>(pairs_.size()) >= params_.min_pairs) {
    fit_ = FitPairs(pairs_, params_.outlier_k);
  } else {
    fit_ = RigidFit();
  }
  ++fit_serial_;
}

double Aligner::required_spread() const
{
  return std::max(params_.min_spread_m, params_.spread_sigma_factor * fit_.sigma);
}

double Aligner::need_m() const
{
  if (word_ != Word::kAligning) {
    return 0.0;
  }
  return std::max(0.0, required_spread() - fit_.spread);
}

double Aligner::err_m() const
{
  if (!alignment_) {
    return 0.0;
  }
  return std::max(alignment_->rms_m, ErrFloor(alignment_->fix, params_.floors));
}

Aligner::Fresh Aligner::FreshState() const
{
  if (!fit_.valid || fit_.inliers < params_.min_pairs) {
    return Fresh::kNone;
  }
  if (fit_.spread < required_spread()) {
    return Fresh::kShort;
  }
  if (fit_.rms <= params_.max_rms_m && std::abs(fit_.scale - 1.0) <= params_.max_scale_error) {
    return Fresh::kGood;
  }
  return Fresh::kBad;
}

Alignment Aligner::FromFit(double now) const
{
  Alignment a;
  const LatLon origin = EnuToLatLon(*origin_, {fit_.tx, fit_.ty});
  a.geo.lat0 = origin.lat;
  a.geo.lon0 = origin.lon;
  a.geo.yaw = std::atan2(std::sin(fit_.yaw), std::cos(fit_.yaw));
  a.rms_m = fit_.rms;
  a.extent_m = fit_.spread;
  a.pairs = fit_.inliers;
  a.scale = fit_.scale;
  a.fix = fit_.cls;
  a.fitted_at = now;
  a.source = Source::kFit;
  return a;
}

double Aligner::FarEdgeMove(const Alignment & a, const Alignment & b) const
{
  double box[4];
  if (has_bounds_) {
    std::copy(bounds_, bounds_ + 4, box);
  } else if (!pairs_.empty()) {
    box[0] = box[2] = pairs_[0].x;
    box[1] = box[3] = pairs_[0].y;
    for (const Pair & p : pairs_) {
      box[0] = std::min(box[0], p.x);
      box[1] = std::min(box[1], p.y);
      box[2] = std::max(box[2], p.x);
      box[3] = std::max(box[3], p.y);
    }
  } else {
    box[0] = box[1] = -50.0;
    box[2] = box[3] = 50.0;
  }
  double worst = 0.0;
  const double ca = std::cos(a.geo.yaw), sa = std::sin(a.geo.yaw);
  for (double x : {box[0], box[2]}) {
    for (double y : {box[1], box[3]}) {
      const LatLon ll = MapToLatLon(b.geo, x, y);
      const EastNorth en = LatLonToEnu({a.geo.lat0, a.geo.lon0}, ll);
      worst = std::max(worst, std::hypot(en.e - (ca * x - sa * y), en.n - (sa * x + ca * y)));
    }
  }
  return worst;
}

void Aligner::SetNumbers(const Alignment & next)
{
  if (!alignment_ || !version_ref_) {
    ++version_;
    version_ref_ = next;
  } else if (FarEdgeMove(*version_ref_, next) > params_.version_move_m) {
    ++version_;
    version_ref_ = next;
  }
  alignment_ = next;
}

void Aligner::Enter(Word word)
{
  word_ = word;
  candidate_.reset();
  if (word == Word::kHeld) {
    agree_since_.reset();
    disagree_since_.reset();
  }
}

void Aligner::Tick(double now)
{
  const Fresh fresh = FreshState();
  const bool disagreeing = alignment_ && disagree_since_ && last_fix_t_ &&
                           *last_fix_t_ - *disagree_since_ >= params_.disagree_s;

  if (word_ == Word::kHeld) {
    // In held the numbers do not change (§5.4).
    if (disagreeing) {
      Enter(Word::kPoor);
    } else if (
      agree_since_ && last_fix_t_ && *last_fix_t_ - *agree_since_ >= params_.held_recover_s &&
      now - *last_fix_t_ <= params_.fix_gap_s) {
      Enter(Word::kAligned);
    }
    return;
  }
  if (word_ == Word::kAligned && (!last_fix_t_ || now - *last_fix_t_ >= params_.held_after_s)) {
    // Only aligned becomes held, and at once: no usable fix for held_after_s is its condition.
    Enter(Word::kHeld);
    return;
  }

  Word candidate;
  if (disagreeing) {
    candidate = Word::kPoor;
  } else if (fresh == Fresh::kGood) {
    candidate = Word::kAligned;
  } else if (fresh == Fresh::kBad) {
    candidate = Word::kPoor;
  } else if (alignment_ && (word_ == Word::kAligned || word_ == Word::kPoor)) {
    candidate = word_;  // a sidecar's numbers with no fit of its own yet
  } else if (fresh == Fresh::kShort) {
    candidate = Word::kAligning;
  } else {
    candidate = Word::kNotAligned;
  }

  if (candidate == word_) {
    candidate_.reset();
    if (word_ == Word::kAligned && fresh == Fresh::kGood && applied_serial_ != fit_serial_) {
      SetNumbers(FromFit(now));  // a refinement; the version moves only past version_move_m
      applied_serial_ = fit_serial_;
    }
    return;
  }
  if (!candidate_ || *candidate_ != candidate) {
    candidate_ = candidate;
    candidate_since_ = now;
  }
  const double wait = Rank(candidate) > Rank(word_) ? params_.better_s : params_.worse_s;
  if (now - candidate_since_ < wait) {
    return;
  }
  if (candidate == Word::kAligned && fresh == Fresh::kGood) {
    SetNumbers(FromFit(now));
    applied_serial_ = fit_serial_;
  } else if (candidate == Word::kPoor && !alignment_ && fresh == Fresh::kBad) {
    SetNumbers(FromFit(now));
    applied_serial_ = fit_serial_;
  }
  Enter(candidate);
}

}  // namespace husarion_ugv_map_gateway
