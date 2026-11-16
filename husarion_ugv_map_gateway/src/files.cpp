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

#include "husarion_ugv_map_gateway/files.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <regex>
#include <sstream>

#include "husarion_ugv_map_gateway/codec.hpp"
#include "husarion_ugv_map_gateway/products.hpp"

namespace husarion_ugv_map_gateway
{

namespace fs = std::filesystem;

namespace
{

std::string Fixed(double value, int decimals)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  return buf;
}

std::string ReadFile(const std::string & path, bool & ok)
{
  std::ifstream in(path, std::ios::binary);
  ok = static_cast<bool>(in);
  std::ostringstream buf;
  if (ok) {
    buf << in.rdbuf();
  }
  return buf.str();
}

// The alignment keys, shared by the sidecar and the session file.
void AlignmentYaml(std::ostringstream & out, const Alignment & a)
{
  out << "lat0: " << Fixed(a.geo.lat0, 9) << "\n";
  out << "lon0: " << Fixed(a.geo.lon0, 9) << "\n";
  out << "yaw: " << Fixed(a.geo.yaw, 9) << "\n";
  out << "rms_m: " << Fixed(a.rms_m, 3) << "\n";
  out << "extent_m: " << Fixed(a.extent_m, 3) << "\n";
  out << "pairs: " << a.pairs << "\n";
  out << "fix: " << FixClassName(a.fix) << "\n";
  out << "fitted_at: " << Fixed(a.fitted_at, 3) << "\n";
}

template <typename T>
std::optional<T> Get(const YAML::Node & node, const char * key)
{
  try {
    if (!node[key] || !node[key].IsScalar()) {
      return std::nullopt;
    }
    return node[key].as<T>();
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

// nullopt: no alignment keys at all. Throws std::runtime_error when they are partial or invalid.
std::optional<Alignment> ParseAlignment(const YAML::Node & node)
{
  if (!node["lat0"] && !node["lon0"] && !node["yaw"]) {
    return std::nullopt;
  }
  const auto lat0 = Get<double>(node, "lat0");
  const auto lon0 = Get<double>(node, "lon0");
  const auto yaw = Get<double>(node, "yaw");
  const auto rms = Get<double>(node, "rms_m");
  const auto extent = Get<double>(node, "extent_m");
  const auto pairs = Get<int>(node, "pairs");
  const auto fix_name = Get<std::string>(node, "fix");
  const auto fitted_at = Get<double>(node, "fitted_at");
  const auto fix = fix_name ? ParseFixClass(*fix_name) : std::nullopt;
  if (
    !lat0 || !lon0 || !yaw || !rms || !extent || !pairs || !fix || !fitted_at ||
    !AllFinite(*lat0, *lon0, *yaw, *rms, *extent, *fitted_at) || std::abs(*lat0) > 90.0 ||
    std::abs(*lon0) > 180.0 || *rms < 0.0 || *extent < 0.0 || *pairs < 0) {
    throw std::runtime_error("incomplete or invalid alignment");
  }
  Alignment a;
  a.geo = {*lat0, *lon0, *yaw};
  a.rms_m = *rms;
  a.extent_m = *extent;
  a.pairs = *pairs;
  a.fix = *fix;
  a.fitted_at = *fitted_at;
  const auto scale = Get<double>(node, "scale");
  a.scale = scale && std::isfinite(*scale) ? *scale : 1.0;
  const auto source = Get<std::string>(node, "source");
  a.source = source && *source == "sidecar" ? Source::kSidecar : Source::kFit;
  return a;
}

}  // namespace

bool ValidSaveName(const std::string & name)
{
  static const std::regex re("^[A-Za-z0-9_-]{1,64}$");
  return std::regex_match(name, re);
}

bool ValidEpoch(const std::string & epoch)
{
  static const std::regex re("^[A-Za-z0-9_-]{1,64}$");
  return std::regex_match(epoch, re);
}

std::string NewEpoch()
{
  std::random_device rd;
  std::uniform_int_distribution<int> digit(0, 15);
  static const char * hex = "0123456789abcdef";
  std::string out;
  for (int i = 0; i < 16; ++i) {
    out.push_back(hex[digit(rd)]);
  }
  return out;
}

std::string SidecarPath(const std::string & map_yaml)
{
  fs::path p(map_yaml);
  return (p.parent_path() / (p.stem().string() + kSidecarSuffix)).string();
}

std::optional<std::string> MapImagePath(const std::string & map_yaml)
{
  try {
    const YAML::Node node = YAML::LoadFile(map_yaml);
    const auto image = Get<std::string>(node, "image");
    if (!image || image->empty()) {
      return std::nullopt;
    }
    fs::path p(*image);
    if (p.is_relative()) {
      p = fs::path(map_yaml).parent_path() / p;
    }
    return p.string();
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

bool WriteFileAtomic(const std::string & path, const std::string & content)
{
  const fs::path target(path);
  const std::string tmp =
    (target.parent_path() / ("." + target.filename().string() + ".tmp")).string();
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    return false;
  }
  size_t done = 0;
  while (done < content.size()) {
    const ssize_t n = ::write(fd, content.data() + done, content.size() - done);
    if (n <= 0) {
      ::close(fd);
      ::unlink(tmp.c_str());
      return false;
    }
    done += static_cast<size_t>(n);
  }
  if (::fsync(fd) != 0 || ::close(fd) != 0) {
    ::unlink(tmp.c_str());
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    ::unlink(tmp.c_str());
    return false;
  }
  return true;
}

std::string SidecarYaml(const Sidecar & sidecar)
{
  std::ostringstream out;
  out << "# Written by husarion_ugv_map_gateway (SPEC-MAP-NAV-2026-11 §5.5).\n";
  out << "schema: " << sidecar.schema << "\n";
  out << "epoch: \"" << sidecar.epoch << "\"\n";
  out << "map_sha256: \"" << sidecar.map_sha256 << "\"\n";
  if (sidecar.alignment) {
    AlignmentYaml(out, *sidecar.alignment);
  }
  if (sidecar.saved_pose) {
    out << "saved_pose:\n";
    out << "  x: " << Fixed(sidecar.saved_pose->x, 6) << "\n";
    out << "  y: " << Fixed(sidecar.saved_pose->y, 6) << "\n";
    out << "  yaw: " << Fixed(sidecar.saved_pose->yaw, 6) << "\n";
  }
  return out.str();
}

std::optional<Sidecar> ParseSidecar(const std::string & text)
{
  try {
    const YAML::Node node = YAML::Load(text);
    if (!node.IsMap()) {
      return std::nullopt;
    }
    Sidecar s;
    const auto schema = Get<int>(node, "schema");
    const auto epoch = Get<std::string>(node, "epoch");
    const auto sha = Get<std::string>(node, "map_sha256");
    if (!schema || *schema != kSidecarSchema || !epoch || !ValidEpoch(*epoch) || !sha) {
      return std::nullopt;
    }
    s.epoch = *epoch;
    s.map_sha256 = *sha;
    s.alignment = ParseAlignment(node);
    if (s.alignment) {
      s.alignment->source = Source::kSidecar;
    }
    const YAML::Node pose = node["saved_pose"];
    if (pose && pose.IsMap()) {
      const auto x = Get<double>(pose, "x");
      const auto y = Get<double>(pose, "y");
      const auto yaw = Get<double>(pose, "yaw");
      if (x && y && yaw && AllFinite(*x, *y, *yaw)) {
        s.saved_pose = SavedPose{*x, *y, *yaw};
      }
    }
    return s;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool WriteSidecar(const std::string & path, const Sidecar & sidecar)
{
  return WriteFileAtomic(path, SidecarYaml(sidecar));
}

std::optional<Sidecar> ReadSidecar(const std::string & path)
{
  bool ok = false;
  const std::string text = ReadFile(path, ok);
  return ok ? ParseSidecar(text) : std::nullopt;
}

std::optional<Sidecar> LoadMatchingSidecar(const std::string & map_yaml)
{
  const auto sidecar = ReadSidecar(SidecarPath(map_yaml));
  if (!sidecar) {
    return std::nullopt;
  }
  const auto image = MapImagePath(map_yaml);
  if (!image) {
    return std::nullopt;
  }
  const auto digest = Sha256File(*image);
  if (!digest || *digest != sidecar->map_sha256) {
    return std::nullopt;
  }
  return sidecar;
}

std::string SessionYaml(const Session & session)
{
  std::ostringstream out;
  out << "# husarion_ugv_map_gateway session (SPEC-MAP-NAV-2026-11 §5.5).\n";
  out << "epoch: \"" << session.epoch << "\"\n";
  out << "gid: \"" << session.gid << "\"\n";
  out << "word: " << WordName(session.word) << "\n";
  out << "version: " << session.version << "\n";
  if (session.alignment) {
    AlignmentYaml(out, *session.alignment);
    out << "scale: " << Fixed(session.alignment->scale, 6) << "\n";
    out << "source: " << SourceName(session.alignment->source) << "\n";
  }
  return out.str();
}

std::optional<Session> ParseSession(const std::string & text)
{
  try {
    const YAML::Node node = YAML::Load(text);
    if (!node.IsMap()) {
      return std::nullopt;
    }
    Session s;
    const auto epoch = Get<std::string>(node, "epoch");
    const auto gid = Get<std::string>(node, "gid");
    const auto word = Get<std::string>(node, "word");
    const auto version = Get<uint32_t>(node, "version");
    if (!epoch || !ValidEpoch(*epoch) || !gid || !word || !ParseWord(*word) || !version) {
      return std::nullopt;
    }
    s.epoch = *epoch;
    s.gid = *gid;
    s.word = *ParseWord(*word);
    s.version = *version;
    s.alignment = ParseAlignment(node);
    return s;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool WriteSession(const std::string & path, const Session & session)
{
  return WriteFileAtomic(path, SessionYaml(session));
}

std::optional<Session> ReadSession(const std::string & path)
{
  bool ok = false;
  const std::string text = ReadFile(path, ok);
  return ok ? ParseSession(text) : std::nullopt;
}

}  // namespace husarion_ugv_map_gateway
