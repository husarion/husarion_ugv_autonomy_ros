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

#ifndef HUSARION_UGV_MAP_GATEWAY_FILES_HPP_
#define HUSARION_UGV_MAP_GATEWAY_FILES_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "husarion_ugv_map_gateway/align.hpp"

namespace husarion_ugv_map_gateway
{

constexpr int kSidecarSchema = 1;
constexpr const char * kSidecarSuffix = ".georef.yaml";

// §5.5 the save name: one path component of [A-Za-z0-9_-], 1 to 64 characters.
bool ValidSaveName(const std::string & name);

// An epoch id: a short printable token (§5.2: `epoch` is a short printable string).
bool ValidEpoch(const std::string & epoch);
// A new random epoch: 16 lowercase hex digits.
std::string NewEpoch();

// `<dir>/<stem>.georef.yaml` beside `<dir>/<stem>.yaml`, as R1's mapnav_params.sidecar_path.
std::string SidecarPath(const std::string & map_yaml);
// The image `map.yaml` names, resolved against its directory; nullopt if it names none.
std::optional<std::string> MapImagePath(const std::string & map_yaml);

struct SavedPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Sidecar
{
  int schema = kSidecarSchema;
  std::string epoch;
  std::string map_sha256;
  std::optional<Alignment> alignment;  // none: a save made before any fit (§5.5)
  std::optional<SavedPose> saved_pose;
};

// Written atomically: a temporary file in the same directory, flushed, then renamed over.
bool WriteFileAtomic(const std::string & path, const std::string & content);

std::string SidecarYaml(const Sidecar & sidecar);
std::optional<Sidecar> ParseSidecar(const std::string & text);
bool WriteSidecar(const std::string & path, const Sidecar & sidecar);
std::optional<Sidecar> ReadSidecar(const std::string & path);

// The sidecar of `map_yaml` if it exists, parses and its `map_sha256` matches the image the map
// names; otherwise nullopt, and the map gets a new epoch (§5.5).
std::optional<Sidecar> LoadMatchingSidecar(const std::string & map_yaml);

// The session file (§5.5): the epoch, the map publisher's GID and the current fit.
struct Session
{
  std::string epoch;
  std::string gid;
  Word word = Word::kNotAligned;
  uint32_t version = 0;
  std::optional<Alignment> alignment;
};
std::string SessionYaml(const Session & session);
std::optional<Session> ParseSession(const std::string & text);
bool WriteSession(const std::string & path, const Session & session);
std::optional<Session> ReadSession(const std::string & path);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_FILES_HPP_
