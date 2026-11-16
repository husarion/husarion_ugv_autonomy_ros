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

#include "husarion_ugv_map_gateway/epoch.hpp"

namespace husarion_ugv_map_gateway
{

const char * EpochWhyName(EpochTracker::Why why)
{
  switch (why) {
    case EpochTracker::Why::kSidecar:
      return "sidecar";
    case EpochTracker::Why::kSession:
      return "session";
    case EpochTracker::Why::kNew:
      return "new";
    case EpochTracker::Why::kPublisherChanged:
      return "publisher_changed";
    case EpochTracker::Why::kMapFileChanged:
      return "map_file_changed";
    default:
      return "none";
  }
}

EpochTracker::EpochTracker(const Params & params, std::function<std::string()> new_epoch)
: params_(params), new_epoch_(std::move(new_epoch))
{
}

void EpochTracker::Decide(const std::string & epoch, Why why)
{
  epoch_ = epoch;
  why_ = why;
}

void EpochTracker::Start(
  const std::string & map_file, const std::optional<Sidecar> & sidecar,
  const std::optional<Session> & session, double now)
{
  started_ = now;
  map_file_ = map_file;
  saved_pair_ = !map_file.empty() && sidecar.has_value();
  if (saved_pair_) {
    Decide(sidecar->epoch, Why::kSidecar);
  } else if (session && !session->gid.empty()) {
    session_ = session;  // decided by the first publisher seen, or new after session_wait_s
  } else {
    Decide(new_epoch_(), Why::kNew);
  }
}

bool EpochTracker::OnPublisher(const std::string & gid, double now)
{
  if (gid.empty()) {
    return false;  // a gap: slam_toolbox or map_server is restarting, or not up yet
  }
  if (!decided()) {
    if (session_ && session_->gid == gid) {
      Decide(session_->epoch, Why::kSession);
      restored_ = session_;
    } else {
      Decide(new_epoch_(), Why::kNew);
    }
    session_.reset();
    gid_ = gid;
    return true;
  }
  if (gid_.empty()) {
    gid_ = gid;  // the first publisher of this epoch
    return false;
  }
  if (gid == gid_) {
    pending_gid_.reset();
    return false;
  }
  if (saved_pair_ && reference_grid_) {
    if (!pending_gid_ || *pending_gid_ != gid) {
      pending_gid_ = gid;
      pending_since_ = now;
    }
    return false;  // decided by its first grid (OnGrid) or the wait (OnTick)
  }
  Decide(new_epoch_(), Why::kPublisherChanged);
  gid_ = gid;
  reference_grid_.reset();
  return true;
}

bool EpochTracker::OnGrid(uint64_t grid_hash, double)
{
  if (!decided()) {
    return false;
  }
  if (pending_gid_) {
    const std::string gid = *pending_gid_;
    pending_gid_.reset();
    gid_ = gid;
    if (reference_grid_ && *reference_grid_ == grid_hash) {
      return false;  // map_server restarted on the same map: same epoch
    }
    Decide(new_epoch_(), Why::kPublisherChanged);
    reference_grid_ = saved_pair_ ? std::optional<uint64_t>(grid_hash) : std::nullopt;
    saved_pair_ = false;  // the new grid is not the saved pair's
    return true;
  }
  if (saved_pair_ && !reference_grid_) {
    reference_grid_ = grid_hash;
  }
  return false;
}

bool EpochTracker::OnTick(double now)
{
  if (!decided() && session_ && now - started_ >= params_.session_wait_s) {
    session_.reset();
    Decide(new_epoch_(), Why::kNew);
    return true;
  }
  if (pending_gid_ && now - pending_since_ >= params_.same_map_wait_s) {
    gid_ = *pending_gid_;
    pending_gid_.reset();
    reference_grid_.reset();
    saved_pair_ = false;
    Decide(new_epoch_(), Why::kPublisherChanged);
    return true;
  }
  return false;
}

bool EpochTracker::OnMapFile(
  const std::string & map_file, const std::optional<Sidecar> & sidecar, double)
{
  if (map_file == map_file_) {
    return false;
  }
  map_file_ = map_file;
  saved_pair_ = !map_file.empty() && sidecar.has_value();
  reference_grid_.reset();
  pending_gid_.reset();
  session_.reset();
  // The GID stays bound: map_server may load another map without a restart.
  Decide(saved_pair_ ? sidecar->epoch : new_epoch_(), Why::kMapFileChanged);
  return true;
}

std::optional<Session> EpochTracker::TakeRestored()
{
  std::optional<Session> out = std::move(restored_);
  restored_.reset();
  return out;
}

}  // namespace husarion_ugv_map_gateway
