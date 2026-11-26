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

#include "husarion_ugv_map_gateway/preview.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

#include "husarion_ugv_map_gateway/files.hpp"

namespace husarion_ugv_map_gateway
{

namespace
{

// The next PGM header token, skipping whitespace and # comments.
std::optional<std::string> Token(const std::string & s, size_t & at)
{
  while (at < s.size()) {
    if (std::isspace(static_cast<unsigned char>(s[at]))) {
      ++at;
    } else if (s[at] == '#') {
      while (at < s.size() && s[at] != '\n') {
        ++at;
      }
    } else {
      break;
    }
  }
  const size_t start = at;
  while (at < s.size() && !std::isspace(static_cast<unsigned char>(s[at]))) {
    ++at;
  }
  if (start == at) {
    return std::nullopt;
  }
  return s.substr(start, at - start);
}

std::optional<uint32_t> Number(const std::optional<std::string> & token)
{
  if (!token || token->empty() || token->size() > 9) {
    return std::nullopt;
  }
  for (char c : *token) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return std::nullopt;
    }
  }
  return static_cast<uint32_t>(std::stoul(*token));
}

}  // namespace

std::optional<GreyImage> DecodePgm(const std::string & bytes)
{
  size_t at = 0;
  if (Token(bytes, at) != std::optional<std::string>("P5")) {
    return std::nullopt;
  }
  const auto width = Number(Token(bytes, at));
  const auto height = Number(Token(bytes, at));
  const auto maxval = Number(Token(bytes, at));
  if (!width || !height || maxval != 255u || *width == 0 || *height == 0) {
    return std::nullopt;
  }
  ++at;  // the single whitespace byte before the pixels
  const size_t n = size_t(*width) * *height;
  if (at + n > bytes.size()) {
    return std::nullopt;
  }
  GreyImage image;
  image.width = *width;
  image.height = *height;
  image.pixels.assign(bytes.begin() + at, bytes.begin() + at + n);
  return image;
}

std::optional<SavedMap> LoadSavedMap(const std::string & map_yaml)
{
  const auto image_path = MapImagePath(map_yaml);
  if (!image_path) {
    return std::nullopt;
  }
  SavedMap out;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
  bool negate = false;
  try {
    const YAML::Node node = YAML::LoadFile(map_yaml);
    out.grid.resolution = node["resolution"].as<double>();
    const YAML::Node origin = node["origin"];
    if (origin && origin.IsSequence() && origin.size() >= 2) {
      out.origin_x = origin[0].as<double>();
      out.origin_y = origin[1].as<double>();
      out.origin_yaw = origin.size() >= 3 ? origin[2].as<double>() : 0.0;
    }
    if (node["occupied_thresh"]) {
      occupied_thresh = node["occupied_thresh"].as<double>();
    }
    if (node["free_thresh"]) {
      free_thresh = node["free_thresh"].as<double>();
    }
    if (node["negate"]) {
      negate = node["negate"].as<int>() != 0;
    }
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
  if (!(out.grid.resolution > 0.0)) {
    return std::nullopt;
  }

  std::ifstream f(*image_path, std::ios::binary);
  if (!f) {
    return std::nullopt;
  }
  const std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  auto image = DecodePgm(bytes);
  if (!image) {
    image = DecodeGreyPng(bytes);
  }
  if (!image) {
    return std::nullopt;
  }

  out.grid.width = image->width;
  out.grid.height = image->height;
  out.grid.cells.assign(size_t(image->width) * image->height, kUnknown);
  for (uint32_t row = 0; row < image->height; ++row) {
    // image row 0 is the top (the largest y); grid row 0 the smallest y
    const uint32_t grid_row = image->height - 1 - row;
    for (uint32_t col = 0; col < image->width; ++col) {
      const uint8_t v = image->pixels[size_t(row) * image->width + col];
      const double occ = negate ? v / 255.0 : (255.0 - v) / 255.0;
      uint8_t cell = kUnknown;
      if (occ > occupied_thresh) {
        cell = kOccupied;
      } else if (occ < free_thresh) {
        cell = kFree;
      }
      out.grid.cells[size_t(grid_row) * image->width + col] = cell;
    }
  }
  return out;
}

GreyGrid ShrinkToSide(const GreyGrid & grid, uint32_t max_side)
{
  GreyGrid out = grid;
  while (max_side > 0 && (out.width > max_side || out.height > max_side)) {
    out = Downsample(out);
  }
  return out;
}

}  // namespace husarion_ugv_map_gateway
