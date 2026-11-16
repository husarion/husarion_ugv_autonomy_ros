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

#ifndef HUSARION_UGV_MAP_GATEWAY_EPOCH_HPP_
#define HUSARION_UGV_MAP_GATEWAY_EPOCH_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "husarion_ugv_map_gateway/files.hpp"

namespace husarion_ugv_map_gateway
{

// The map epoch rule (§5.5, [06] M6). The epoch changes:
// - when the map topic's publisher GID changes, unless the gateway runs on a saved pair with its
//   sidecar and the new publisher serves the same grid;
// - when `map_file_name` names another map;
// - at a start with neither a saved pair with its sidecar nor a session file whose GID matches.
// It never changes on the grid's origin, size or content. A moment with no publisher is no change.
class EpochTracker
{
public:
  enum class Why {
    kNone,
    kSidecar,           // the saved pair's sidecar
    kSession,           // restored: the session file's GID matched
    kNew,               // a start with neither
    kPublisherChanged,  // another GID
    kMapFileChanged,    // another map_file_name
  };

  struct Params
  {
    double session_wait_s = 30.0;   // how long a start waits for a publisher to match the session
    double same_map_wait_s = 10.0;  // how long a saved pair waits for the new publisher's grid
  };

  explicit EpochTracker(const Params & params, std::function<std::string()> new_epoch = NewEpoch);

  // At start. `sidecar`: the saved pair's matching sidecar, if the gateway runs on one;
  // `session`: the session file, if any.
  void Start(
    const std::string & map_file, const std::optional<Sidecar> & sidecar,
    const std::optional<Session> & session, double now);

  // The current publisher GID of the map topic ("" when none). Returns true when the epoch
  // changed or was decided by this call.
  bool OnPublisher(const std::string & gid, double now);
  // A grid arrived (its content hash). Returns true when the epoch changed.
  bool OnGrid(uint64_t grid_hash, double now);
  // Called periodically: the timeouts of the two waits. Returns true when the epoch changed.
  bool OnTick(double now);
  // `map_file_name` now names `map_file`. Returns true when the epoch changed.
  bool OnMapFile(const std::string & map_file, const std::optional<Sidecar> & sidecar, double now);

  bool decided() const { return !epoch_.empty(); }
  const std::string & epoch() const { return epoch_; }
  const std::string & gid() const { return gid_; }
  const std::string & map_file() const { return map_file_; }
  bool on_saved_pair() const { return saved_pair_; }
  Why why() const { return why_; }
  // The session to restore, set once by a start that matched it (read and cleared by the node).
  std::optional<Session> TakeRestored();

private:
  void Decide(const std::string & epoch, Why why);

  Params params_;
  std::function<std::string()> new_epoch_;
  std::string epoch_;
  std::string gid_;  // the GID bound to this epoch; "" until a publisher was seen
  std::string map_file_;
  bool saved_pair_ = false;  // on a saved pair with its sidecar
  Why why_ = Why::kNone;
  double started_ = 0.0;
  std::optional<Session> session_;   // waiting for a publisher to match it
  std::optional<Session> restored_;  // matched, for the node to restore the fit
  std::optional<uint64_t> reference_grid_;
  std::optional<std::string> pending_gid_;
  double pending_since_ = 0.0;
};

const char * EpochWhyName(EpochTracker::Why why);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_EPOCH_HPP_
