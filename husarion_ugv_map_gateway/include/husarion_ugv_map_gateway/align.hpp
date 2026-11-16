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

#ifndef HUSARION_UGV_MAP_GATEWAY_ALIGN_HPP_
#define HUSARION_UGV_MAP_GATEWAY_ALIGN_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "husarion_ugv_map_gateway/geo.hpp"

namespace husarion_ugv_map_gateway
{

// The alignment state word (§5.4).
enum class Word { kNotAligned, kAligning, kAligned, kPoor, kHeld };
const char * WordName(Word word);
std::optional<Word> ParseWord(const std::string & name);
// `level` of `state`: OK for aligned and held, WARN otherwise.
bool WordIsOk(Word word);
// fromLL and toLL are served only in these words (§5.3).
inline bool WordServesLL(Word word) { return word == Word::kAligned || word == Word::kHeld; }

enum class Source { kFit, kSidecar };
const char * SourceName(Source source);

struct Alignment
{
  GeoRef geo;
  double rms_m = 0.0;
  double extent_m = 0.0;
  int pairs = 0;
  double scale = 1.0;
  FixClass fix = FixClass::kNone;
  double fitted_at = 0.0;  // Unix seconds
  Source source = Source::kFit;
};

// Starting thresholds (§5.4); all are gateway parameters.
struct AlignParams
{
  int min_pairs = 10;
  double min_spread_m = 10.0;
  double spread_sigma_factor = 20.0;
  double max_rms_m = 3.0;
  double max_scale_error = 0.02;
  double disagree_factor = 3.0;  // a fix disagrees when it is over this times err_m off
  double disagree_s = 30.0;
  double worse_s = 5.0;
  double better_s = 10.0;
  double held_after_s = 10.0;
  double held_recover_s = 10.0;
  double version_move_m = 0.2;
  double pair_min_distance_m = 0.5;
  size_t max_pairs = 2000;
  double fix_gap_s = 3.0;  // fixes further apart than this end a run of agreeing fixes
  double outlier_k = 3.0;
  ErrFloors floors;
};

struct FixInput
{
  double t = 0.0;  // the fix's stamp, seconds
  LatLon ll;
  FixQuality quality;
  bool has_map = false;  // the antenna's place in the map frame at t resolved
  double x = 0.0;
  double y = 0.0;
};

// The fit and the state word with its hysteresis (§5.4, D12). Time is passed in, so the rules are
// tested without clocks.
class Aligner
{
public:
  explicit Aligner(const AlignParams & params = AlignParams());

  // A new epoch: no pairs, no alignment, `not_aligned`. The version goes up if there was one.
  void Reset();
  // A sidecar's alignment on a saved pair: `held`, source `sidecar` (§5.5).
  void Load(const Alignment & alignment);
  // The session file's fit, for a gateway that restarts alone: an aligned or held fit comes back
  // as `held` (no fix has been seen yet), a poor one as `poor`; anything else is dropped.
  void Restore(Word word, const std::optional<Alignment> & alignment, uint32_t version);
  // The map's corners in the map frame, for the version's far-edge rule.
  void SetMapBounds(double min_x, double min_y, double max_x, double max_y);

  void AddFix(const FixInput & fix);
  void Tick(double now);

  Word word() const { return word_; }
  const std::optional<Alignment> & alignment() const { return alignment_; }
  uint32_t version() const { return version_; }
  const RigidFit & fit() const { return fit_; }
  size_t pair_count() const { return pairs_.size(); }
  // §5.4: in `aligning` the spread still missing, else 0.
  double need_m() const;
  // The spread the fit needs: the larger of min_spread_m and spread_sigma_factor x sigma.
  double required_spread() const;
  // err_m: the larger of rms_m and the floor of the fix class.
  double err_m() const;
  std::optional<double> last_fix_time() const { return last_fix_t_; }
  FixClass last_fix_class() const { return last_fix_class_; }

private:
  enum class Fresh { kNone, kShort, kGood, kBad };
  Fresh FreshState() const;
  // Whether the newest fix agrees with the fit of the pairs (not with the published numbers).
  bool NewestFixFitsTheFit() const;
  Alignment FromFit(double now) const;
  void SetNumbers(const Alignment & next);
  double FarEdgeMove(const Alignment & a, const Alignment & b) const;
  void Enter(Word word);
  static int Rank(Word word);

  AlignParams params_;
  Word word_ = Word::kNotAligned;
  std::optional<Alignment> alignment_;
  std::optional<Alignment> version_ref_;
  uint32_t version_ = 0;

  std::optional<LatLon> origin_;  // this epoch's east-north origin: its first usable fix
  std::vector<Pair> pairs_;
  RigidFit fit_;
  uint64_t fit_serial_ = 0;
  uint64_t applied_serial_ = 0;

  std::optional<double> last_fix_t_;
  FixClass last_fix_class_ = FixClass::kNone;
  std::optional<FixInput> last_fix_;  // the newest fix with a place in the map
  std::optional<double> agree_since_;
  std::optional<double> disagree_since_;
  std::optional<Word> candidate_;
  double candidate_since_ = 0.0;
  bool has_bounds_ = false;
  double bounds_[4] = {0, 0, 0, 0};
};

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_ALIGN_HPP_
