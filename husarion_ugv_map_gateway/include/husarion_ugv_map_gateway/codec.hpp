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

#ifndef HUSARION_UGV_MAP_GATEWAY_CODEC_HPP_
#define HUSARION_UGV_MAP_GATEWAY_CODEC_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace husarion_ugv_map_gateway
{

// SHA-256 of a byte string, as 64 lowercase hex digits (the sidecar's `map_sha256`).
std::string Sha256Hex(const std::string & data);

// SHA-256 of a file's bytes; nullopt when it cannot be read.
std::optional<std::string> Sha256File(const std::string & path);

// An 8-bit greyscale PNG. `pixels` holds `height` rows of `width` bytes, row 0 first (the top).
std::string EncodeGreyPng(const std::vector<uint8_t> & pixels, uint32_t width, uint32_t height);

// The inverse, for tests: nullopt unless `png` is an 8-bit greyscale, non-interlaced PNG.
struct GreyImage
{
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<uint8_t> pixels;
};
std::optional<GreyImage> DecodeGreyPng(const std::string & png);

// 64-bit FNV-1a, used to tell whether a grid changed (§5.1 "on change only").
uint64_t Fnv1a64(const void * data, size_t size, uint64_t seed = 1469598103934665603ULL);

}  // namespace husarion_ugv_map_gateway

#endif  // HUSARION_UGV_MAP_GATEWAY_CODEC_HPP_
