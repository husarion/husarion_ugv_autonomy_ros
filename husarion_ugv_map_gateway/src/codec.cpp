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

#include "husarion_ugv_map_gateway/codec.hpp"

#include <zlib.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace husarion_ugv_map_gateway
{

namespace
{

constexpr std::array<uint32_t, 64> kSha256K = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

void Sha256Block(std::array<uint32_t, 8> & h, const uint8_t * block)
{
  std::array<uint32_t, 64> w{};
  for (int i = 0; i < 16; ++i) {
    w[i] = (uint32_t(block[4 * i]) << 24) | (uint32_t(block[4 * i + 1]) << 16) |
           (uint32_t(block[4 * i + 2]) << 8) | uint32_t(block[4 * i + 3]);
  }
  for (int i = 16; i < 64; ++i) {
    const uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
  for (int i = 0; i < 64; ++i) {
    const uint32_t s1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
    const uint32_t ch = (e & f) ^ (~e & g);
    const uint32_t t1 = hh + s1 + ch + kSha256K[i] + w[i];
    const uint32_t s0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t t2 = s0 + maj;
    hh = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h[0] += a;
  h[1] += b;
  h[2] += c;
  h[3] += d;
  h[4] += e;
  h[5] += f;
  h[6] += g;
  h[7] += hh;
}

void PutU32(std::string & out, uint32_t v)
{
  out.push_back(char((v >> 24) & 0xff));
  out.push_back(char((v >> 16) & 0xff));
  out.push_back(char((v >> 8) & 0xff));
  out.push_back(char(v & 0xff));
}

uint32_t GetU32(const std::string & in, size_t at)
{
  return (uint32_t(uint8_t(in[at])) << 24) | (uint32_t(uint8_t(in[at + 1])) << 16) |
         (uint32_t(uint8_t(in[at + 2])) << 8) | uint32_t(uint8_t(in[at + 3]));
}

void PutChunk(std::string & out, const char * type, const std::string & data)
{
  PutU32(out, static_cast<uint32_t>(data.size()));
  std::string body(type, 4);
  body += data;
  out += body;
  PutU32(out, crc32(0L, reinterpret_cast<const Bytef *>(body.data()), body.size()));
}

constexpr char kPngSignature[] = "\x89PNG\r\n\x1a\n";

}  // namespace

std::string Sha256Hex(const std::string & data)
{
  std::array<uint32_t, 8> h = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                               0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  const size_t full = data.size() / 64;
  const auto * bytes = reinterpret_cast<const uint8_t *>(data.data());
  for (size_t i = 0; i < full; ++i) {
    Sha256Block(h, bytes + 64 * i);
  }
  std::array<uint8_t, 128> tail{};
  const size_t rest = data.size() - 64 * full;
  std::memcpy(tail.data(), bytes + 64 * full, rest);
  tail[rest] = 0x80;
  const size_t tail_len = rest + 1 + 8 <= 64 ? 64 : 128;
  const uint64_t bits = uint64_t(data.size()) * 8;
  for (int i = 0; i < 8; ++i) {
    tail[tail_len - 1 - i] = uint8_t((bits >> (8 * i)) & 0xff);
  }
  for (size_t i = 0; i < tail_len; i += 64) {
    Sha256Block(h, tail.data() + i);
  }
  static const char * hex = "0123456789abcdef";
  std::string out;
  for (uint32_t v : h) {
    for (int s = 28; s >= 0; s -= 4) {
      out.push_back(hex[(v >> s) & 0xf]);
    }
  }
  return out;
}

std::optional<std::string> Sha256File(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream buf;
  buf << in.rdbuf();
  if (in.bad()) {
    return std::nullopt;
  }
  return Sha256Hex(buf.str());
}

std::string EncodeGreyPng(const std::vector<uint8_t> & pixels, uint32_t width, uint32_t height)
{
  // Each row: filter type 0 (None), then the row's bytes.
  std::string raw;
  raw.reserve(size_t(height) * (width + 1));
  for (uint32_t row = 0; row < height; ++row) {
    raw.push_back('\0');
    raw.append(reinterpret_cast<const char *>(pixels.data()) + size_t(row) * width, width);
  }
  uLongf bound = compressBound(raw.size());
  std::string deflated(bound, '\0');
  compress2(
    reinterpret_cast<Bytef *>(deflated.data()), &bound, reinterpret_cast<const Bytef *>(raw.data()),
    raw.size(), Z_BEST_COMPRESSION);
  deflated.resize(bound);

  std::string ihdr;
  PutU32(ihdr, width);
  PutU32(ihdr, height);
  ihdr.push_back(char(8));  // bit depth
  ihdr.push_back(char(0));  // colour type: greyscale
  ihdr.push_back(char(0));  // compression
  ihdr.push_back(char(0));  // filter method
  ihdr.push_back(char(0));  // no interlace

  std::string out(kPngSignature, 8);
  PutChunk(out, "IHDR", ihdr);
  PutChunk(out, "IDAT", deflated);
  PutChunk(out, "IEND", "");
  return out;
}

std::optional<GreyImage> DecodeGreyPng(const std::string & png)
{
  if (png.size() < 8 || png.compare(0, 8, std::string(kPngSignature, 8)) != 0) {
    return std::nullopt;
  }
  GreyImage image;
  std::string idat;
  size_t at = 8;
  bool header = false;
  while (at + 12 <= png.size()) {
    const uint32_t len = GetU32(png, at);
    if (at + 12 + len > png.size()) {
      return std::nullopt;
    }
    const std::string type = png.substr(at + 4, 4);
    const std::string data = png.substr(at + 8, len);
    const uint32_t crc = GetU32(png, at + 8 + len);
    const std::string body = png.substr(at + 4, 4 + len);
    if (crc != crc32(0L, reinterpret_cast<const Bytef *>(body.data()), body.size())) {
      return std::nullopt;
    }
    if (type == "IHDR") {
      if (len != 13 || data[8] != 8 || data[9] != 0 || data[12] != 0) {
        return std::nullopt;
      }
      image.width = GetU32(data, 0);
      image.height = GetU32(data, 4);
      header = true;
    } else if (type == "IDAT") {
      idat += data;
    } else if (type == "IEND") {
      break;
    }
    at += 12 + len;
  }
  if (!header) {
    return std::nullopt;
  }
  const size_t stride = size_t(image.width) + 1;
  uLongf raw_len = stride * image.height;
  std::string raw(raw_len, '\0');
  if (
    uncompress(
      reinterpret_cast<Bytef *>(raw.data()), &raw_len, reinterpret_cast<const Bytef *>(idat.data()),
      idat.size()) != Z_OK ||
    raw_len != stride * image.height) {
    return std::nullopt;
  }
  image.pixels.assign(size_t(image.width) * image.height, 0);
  for (uint32_t row = 0; row < image.height; ++row) {
    const uint8_t filter = uint8_t(raw[row * stride]);
    for (uint32_t col = 0; col < image.width; ++col) {
      const int x = uint8_t(raw[row * stride + 1 + col]);
      const int a = col > 0 ? image.pixels[size_t(row) * image.width + col - 1] : 0;
      const int b = row > 0 ? image.pixels[size_t(row - 1) * image.width + col] : 0;
      const int c = (row > 0 && col > 0) ? image.pixels[size_t(row - 1) * image.width + col - 1]
                                         : 0;
      int value = x;
      switch (filter) {
        case 0:
          break;
        case 1:
          value = x + a;
          break;
        case 2:
          value = x + b;
          break;
        case 3:
          value = x + (a + b) / 2;
          break;
        case 4: {
          const int p = a + b - c;
          const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
          value = x + ((pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c));
          break;
        }
        default:
          return std::nullopt;
      }
      image.pixels[size_t(row) * image.width + col] = uint8_t(value & 0xff);
    }
  }
  return image;
}

uint64_t Fnv1a64(const void * data, size_t size, uint64_t seed)
{
  uint64_t h = seed;
  const auto * p = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; ++i) {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

}  // namespace husarion_ugv_map_gateway
