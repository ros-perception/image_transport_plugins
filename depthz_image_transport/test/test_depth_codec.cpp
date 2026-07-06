// Copyright (c) 2026, Davide Faconti
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <zstd.h>

#include <cstring>
#include <random>
#include <type_traits>
#include <stdexcept>
#include <vector>

#include "depth_codec.hpp"

namespace
{

float from_bits(uint32_t u)
{
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

// Overload shims so the round-trip harness below is pixel-format generic.
void codec_encode(
  const float * data, uint32_t w, uint32_t h, std::vector<uint8_t> & out, int level)
{
  depth_codec::encode_depth(data, w, h, out, level);
}
void codec_encode(
  const uint16_t * data, uint32_t w, uint32_t h, std::vector<uint8_t> & out, int level)
{
  depth_codec::encode_depth16(data, w, h, out, level);
}
void codec_decode(const uint8_t * blob, size_t size, float * out)
{
  depth_codec::decode_depth(blob, size, out);
}
void codec_decode(const uint8_t * blob, size_t size, uint16_t * out)
{
  depth_codec::decode_depth16(blob, size, out);
}

// The documented qpred contract: +/- step/2 plus a few float ULPs of the
// value (reconstruction and the step itself are float-rounded).
float qpred_tol(float step, float v)
{
  return 0.5f * step + 4.0f * std::numeric_limits<float>::epsilon() * v;
}

template<typename T>
constexpr depth_codec::PixelFormat expected_format()
{
  return std::is_same_v<T, float> ?
         depth_codec::PixelFormat::FLOAT32 : depth_codec::PixelFormat::UINT16;
}

// Encode + read_header + decode into a preallocated buffer, and require a
// BIT-EXACT payload (memcmp over the raw bytes, so NaN payloads, +/-inf and
// -0.0 are checked too).
template<typename T>
void expect_roundtrip(const std::vector<T> & img, uint32_t w, uint32_t h)
{
  std::vector<uint8_t> blob;  // reused across levels: capacity-reuse path
  for (int level : {1, 3}) {
    codec_encode(img.data(), w, h, blob, level);
    const depth_codec::BlobHeader header =
      depth_codec::read_header(blob.data(), blob.size());
    ASSERT_EQ(header.width, w);
    ASSERT_EQ(header.height, h);
    ASSERT_EQ(header.format, expected_format<T>());
    std::vector<T> back(img.size());
    codec_decode(blob.data(), blob.size(), back.data());
    if (!img.empty()) {
      ASSERT_EQ(0, std::memcmp(back.data(), img.data(), img.size() * sizeof(T)));
    }
  }
}

std::vector<float> make_depth_frame(uint32_t w, uint32_t h)
{
  std::vector<float> img(static_cast<size_t>(w) * h);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      const int q = 1 + static_cast<int>((x * 131 + y * 17) % 5000);
      img[static_cast<size_t>(y) * w + x] = 10100.0f / (static_cast<float>(q) + 1009.0f);
    }
  }
  // Canonical-NaN hole, like an invalid region from a depth camera.
  for (uint32_t y = h / 4; y < h / 2; ++y) {
    for (uint32_t x = w / 3; x < 2 * w / 3; ++x) {
      img[static_cast<size_t>(y) * w + x] = from_bits(0x7FC00000u);
    }
  }
  return img;
}

std::vector<uint16_t> make_depth_frame16(uint32_t w, uint32_t h)
{
  std::vector<uint16_t> img(static_cast<size_t>(w) * h);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      img[static_cast<size_t>(y) * w + x] =
        static_cast<uint16_t>(300 + (x * 131 + y * 17) % 5000);
    }
  }
  // Zero hole, the 16UC1 invalid-pixel convention.
  for (uint32_t y = h / 4; y < h / 2; ++y) {
    for (uint32_t x = w / 3; x < 2 * w / 3; ++x) {
      img[static_cast<size_t>(y) * w + x] = 0;
    }
  }
  return img;
}

// ---- helpers to hand-craft adversarial blobs --------------------------------
// The public blob layout ('D''P''C''1' | u8 name_len | name | i32 level |
// u32 w | u32 h | payload) is documented in depth_codec.hpp; building it by
// hand here (rather than via encode_depth/encode_depth16) is how the
// decoder's robustness against malformed/hostile input is exercised.
void append_le(std::vector<uint8_t> & v, uint64_t x, int nbytes)
{
  for (int i = 0; i < nbytes; ++i) {
    v.push_back(static_cast<uint8_t>(x >> (8 * i)));
  }
}

std::vector<uint8_t> make_blob_header(const char * method, uint32_t level, uint32_t w, uint32_t h)
{
  std::vector<uint8_t> blob;
  const char magic[4] = {'D', 'P', 'C', '1'};
  blob.insert(blob.end(), magic, magic + 4);
  const size_t name_len = std::strlen(method);
  blob.push_back(static_cast<uint8_t>(name_len));
  blob.insert(blob.end(), method, method + name_len);
  append_le(blob, level, 4);
  append_le(blob, w, 4);
  append_le(blob, h, 4);
  return blob;
}

// A real zstd frame that compresses `plain` verbatim (so its declared
// content size is honest -- only the decompressed *content* is adversarial).
std::vector<uint8_t> zstd_compress_raw(const std::vector<uint8_t> & plain)
{
  const size_t bound = ZSTD_compressBound(plain.size());
  std::vector<uint8_t> out(bound);
  const size_t k = ZSTD_compress(out.data(), bound, plain.data(), plain.size(), 1);
  out.resize(k);
  return out;
}

// A minimal, hand-built zstd frame header (magic + Single_Segment frame
// descriptor + an 8-byte Frame_Content_Size field) that declares an
// enormous decompressed size. It never needs to decompress successfully --
// zstd_unpack must reject it from the header alone, before allocating
// anything sized off that claim.
std::vector<uint8_t> fake_huge_zstd_frame()
{
  std::vector<uint8_t> b;
  append_le(b, 0xFD2FB528u, 4);  // zstd magic number
  // Frame_Header_Descriptor: Frame_Content_Size_flag=3 (8-byte field),
  // Single_Segment_flag=1 (so Window_Descriptor is omitted).
  b.push_back(static_cast<uint8_t>((3u << 6) | (1u << 5)));
  append_le(b, 1ULL << 40, 8);  // ~1 TiB claimed content size
  b.push_back(0);
  b.push_back(0);
  return b;
}

}  // namespace

TEST(DepthCodec, depth_frame_roundtrip_32f)
{
  expect_roundtrip(make_depth_frame(97, 53), 97, 53);
  expect_roundtrip(make_depth_frame(640, 480), 640, 480);
}

TEST(DepthCodec, depth_frame_roundtrip_16u)
{
  expect_roundtrip(make_depth_frame16(97, 53), 97, 53);
  expect_roundtrip(make_depth_frame16(640, 480), 640, 480);
}

TEST(DepthCodec, special_values_32f)
{
  const uint32_t bits[] = {
    0x00000000u,  // +0.0
    0x80000000u,  // -0.0
    0x7F800000u,  // +inf
    0xFF800000u,  // -inf
    0x7FC00000u,  // canonical quiet NaN
    0x7FC00001u,  // NaN, non-canonical payload
    0x7F800001u,  // signalling NaN
    0xFFC00000u,  // negative NaN
    0xFFFFFFFFu,  // NaN, all ones
    0x00000001u,  // smallest positive denormal
    0x807FFFFFu,  // negative denormal
    0x7F7FFFFFu,  // FLT_MAX
    0xFF7FFFFFu,  // -FLT_MAX
    0x00800000u,  // FLT_MIN
    0xBF800000u,  // -1.0
    0x3F800000u,  // 1.0
  };
  const uint32_t w = 8;
  const uint32_t h = 8;
  std::vector<float> img(static_cast<size_t>(w) * h);
  for (size_t i = 0; i < img.size(); ++i) {
    img[i] = from_bits(bits[i % std::size(bits)]);
  }
  expect_roundtrip(img, w, h);
}

TEST(DepthCodec, special_values_16u)
{
  // Extremes and large jumps (exercises the mod-2^16 residual wrap).
  const uint16_t vals[] = {0, 65535, 1, 65534, 32768, 32767, 0, 65535};
  const uint32_t w = 8;
  const uint32_t h = 8;
  std::vector<uint16_t> img(static_cast<size_t>(w) * h);
  for (size_t i = 0; i < img.size(); ++i) {
    img[i] = vals[i % std::size(vals)];
  }
  expect_roundtrip(img, w, h);
}

TEST(DepthCodec, dictionary_overflow_falls_back)
{
  // > 65536 distinct bit patterns forces the dictionary-free fpred fallback
  // (32FC1 only: 16UC1 cannot overflow by construction). Still bit-exact.
  std::mt19937 rng(12345);
  const uint32_t w = 512;
  const uint32_t h = 256;
  std::vector<float> img(static_cast<size_t>(w) * h);
  for (auto & f : img) {
    f = from_bits(rng());
  }
  expect_roundtrip(img, w, h);
}

TEST(DepthCodec, lossless_method_selection)
{
  // The blob method name records which path encode_depth took: dictionary
  // ("dpred") for low-cardinality images, "fpred" past 65536 distinct
  // values. The name starts at byte 5 (after magic + name_len).
  const auto method_of = [](const std::vector<uint8_t> & blob) {
      return std::string(reinterpret_cast<const char *>(blob.data() + 5), blob[4]);
    };

  std::vector<uint8_t> blob;
  const auto few = make_depth_frame(200, 100);  // ~5000 distinct values
  depth_codec::encode_depth(few.data(), 200, 100, blob, 1);
  EXPECT_EQ(method_of(blob), "dpred");

  // Full-precision synthetic "stereo" depth: every pixel distinct.
  std::vector<float> many(400 * 300);
  for (size_t i = 0; i < many.size(); ++i) {
    many[i] = 0.15f + static_cast<float>(i) * 1e-5f;
  }
  depth_codec::encode_depth(many.data(), 400, 300, blob, 1);
  EXPECT_EQ(method_of(blob), "fpred");
  std::vector<float> back(many.size());
  depth_codec::decode_depth(blob.data(), blob.size(), back.data());
  EXPECT_EQ(0, std::memcmp(back.data(), many.data(), many.size() * 4));
}

TEST(DepthCodec, degenerate_shapes)
{
  expect_roundtrip(std::vector<float>(64 * 32, 1.25f), 64, 32);  // constant
  expect_roundtrip<float>({0.5f}, 1, 1);
  expect_roundtrip<float>({1.f, 2.f, 3.f, 4.f, 5.f}, 5, 1);
  expect_roundtrip<float>({1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f}, 1, 7);
  expect_roundtrip(std::vector<uint16_t>(64 * 32, 1250), 64, 32);
  expect_roundtrip<uint16_t>({1234}, 1, 1);
  expect_roundtrip<uint16_t>({1, 2, 3, 4, 5}, 5, 1);
  expect_roundtrip<uint16_t>({1, 2, 3, 4, 5, 6, 7}, 1, 7);
}

// Every w/h combination with n == 0 must encode and decode cleanly for
// both pixel formats (no OOB access, no null-pointer memcpy) -- ASan/UBSan
// must stay silent on all of these.
TEST(DepthCodec, degenerate_zero_dimension)
{
  expect_roundtrip<float>({}, 0, 0);
  expect_roundtrip<float>({}, 0, 9);
  expect_roundtrip<float>({}, 9, 0);
  expect_roundtrip<uint16_t>({}, 0, 0);
  expect_roundtrip<uint16_t>({}, 0, 9);
  expect_roundtrip<uint16_t>({}, 9, 0);
}

TEST(DepthCodec, malformed_input_throws)
{
  const uint8_t junk[8] = {'X', 'X', 'X', 'X', 0, 0, 0, 0};
  EXPECT_THROW(depth_codec::read_header(junk, sizeof(junk)), std::runtime_error);

  // Truncated valid blob must throw, not crash.
  const float one = 1.0f;
  std::vector<uint8_t> blob;
  depth_codec::encode_depth(&one, 1, 1, blob, 1);
  float out = 0.0f;
  EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size() / 2, &out),
    std::runtime_error);
}

TEST(DepthCodec, pixel_format_mismatch_throws)
{
  const float onef = 1.0f;
  const uint16_t oneu = 1;
  std::vector<uint8_t> blob32;
  std::vector<uint8_t> blob16;
  depth_codec::encode_depth(&onef, 1, 1, blob32, 1);
  depth_codec::encode_depth16(&oneu, 1, 1, blob16, 1);

  uint16_t out16 = 0;
  float out32 = 0.0f;
  EXPECT_THROW(depth_codec::decode_depth16(blob32.data(), blob32.size(), &out16),
    std::runtime_error);
  EXPECT_THROW(depth_codec::decode_depth(blob16.data(), blob16.size(), &out32),
    std::runtime_error);
}

// A zstd frame whose header claims an enormous decompressed size must be
// rejected before any allocation sized off that claim, for both
// pixel-format decode paths.
TEST(DepthCodec, decoder_rejects_huge_zstd_content_size)
{
  const auto payload = fake_huge_zstd_frame();

  std::vector<uint8_t> blob32 = make_blob_header("dpred", 1, 4, 4);
  blob32.insert(blob32.end(), payload.begin(), payload.end());
  std::vector<float> out32(16);
  EXPECT_THROW(depth_codec::decode_depth(blob32.data(), blob32.size(), out32.data()),
    std::runtime_error);

  std::vector<uint8_t> blob16 = make_blob_header("dpred16", 1, 4, 4);
  blob16.insert(blob16.end(), payload.begin(), payload.end());
  std::vector<uint16_t> out16(16);
  EXPECT_THROW(depth_codec::decode_depth16(blob16.data(), blob16.size(), out16.data()),
    std::runtime_error);
}

// A dpred payload whose mode byte is neither 0 (raw fallback) nor 1
// (dictionary) must be rejected, not silently treated as a known mode.
TEST(DepthCodec, decoder_rejects_invalid_mode_byte)
{
  const std::vector<uint8_t> plain = {5};  // mode 5: neither 0 nor 1
  const auto payload = zstd_compress_raw(plain);
  std::vector<uint8_t> blob = make_blob_header("dpred", 1, 1, 1);
  blob.insert(blob.end(), payload.begin(), payload.end());
  float out = 0.0f;
  EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), &out), std::runtime_error);
}

// Dictionary mode (byte 0 == 1) with an idx_bytes field other than 2 must
// be rejected explicitly.
TEST(DepthCodec, decoder_rejects_invalid_idx_bytes)
{
  std::vector<uint8_t> plain = {1, 3, 0, 0, 0, 0};  // mode=1, idx_bytes=3 (invalid), ds=0
  const auto payload = zstd_compress_raw(plain);
  std::vector<uint8_t> blob = make_blob_header("dpred", 1, 0, 0);
  blob.insert(blob.end(), payload.begin(), payload.end());
  float out = 0.0f;
  EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), &out), std::runtime_error);
}

// ---- qpred (lossy, configurable quantization) --------------------------------

TEST(DepthCodec, qpred_error_bound_and_invalid_pixels)
{
  // Realistic stereo-like depth: every pixel distinct, range 0.15-10 m, a
  // NaN hole plus assorted invalid values. At step = 0.1 mm the range
  // needs ~100k quantization levels -- past 16 bits, exercising the varint
  // path -- and every valid pixel must reconstruct within +/- step/2.
  const uint32_t w = 200;
  const uint32_t h = 150;
  const float step = 0.0001f;
  std::mt19937 rng(999);
  std::uniform_real_distribution<float> depth(0.15f, 10.0f);
  std::vector<float> img(static_cast<size_t>(w) * h);
  for (auto & f : img) {
    f = depth(rng);
  }
  img[0] = from_bits(0x7FC00000u);              // quiet NaN
  img[1] = std::numeric_limits<float>::infinity();
  img[2] = -std::numeric_limits<float>::infinity();
  img[3] = -1.5f;                               // negative: invalid
  img[4] = 0.0f;                                // zero: invalid
  for (size_t i = w; i < 3 * w; ++i) {          // a NaN hole spanning rows
    img[i] = from_bits(0x7FC00000u);
  }

  std::vector<uint8_t> blob;
  depth_codec::encode_depth_quantized(img.data(), w, h, blob, step, 1);

  const depth_codec::BlobHeader header = depth_codec::read_header(blob.data(), blob.size());
  ASSERT_EQ(header.width, w);
  ASSERT_EQ(header.height, h);
  ASSERT_EQ(header.format, depth_codec::PixelFormat::FLOAT32);
  ASSERT_FLOAT_EQ(header.quantization_step, step);

  std::vector<float> back(img.size());
  depth_codec::decode_depth(blob.data(), blob.size(), back.data());
  for (size_t i = 0; i < img.size(); ++i) {
    const bool valid_in = std::isfinite(img[i]) && img[i] > 0.0f;
    if (valid_in) {
      ASSERT_TRUE(std::isfinite(back[i])) << "pixel " << i;
      ASSERT_NEAR(back[i], img[i], qpred_tol(step, img[i])) << "pixel " << i;
    } else {
      ASSERT_TRUE(std::isnan(back[i])) << "pixel " << i;  // REP 118
    }
  }
}

TEST(DepthCodec, qpred_lossless_header_reports_no_quantization)
{
  const auto img = make_depth_frame(64, 48);
  std::vector<uint8_t> blob;
  depth_codec::encode_depth(img.data(), 64, 48, blob, 1);
  const depth_codec::BlobHeader header = depth_codec::read_header(blob.data(), blob.size());
  ASSERT_FLOAT_EQ(header.quantization_step, 0.0f);
}

TEST(DepthCodec, qpred_degenerate_shapes)
{
  const float step = 0.001f;
  for (const auto [w, h] : {std::pair<uint32_t, uint32_t>{0, 0}, {0, 9}, {9, 0}, {1, 1}}) {
    std::vector<float> img(static_cast<size_t>(w) * h, 1.25f);
    std::vector<uint8_t> blob;
    depth_codec::encode_depth_quantized(img.data(), w, h, blob, step, 1);
    std::vector<float> back(img.size());
    depth_codec::decode_depth(blob.data(), blob.size(), back.data());
    for (const float v : back) {
      ASSERT_NEAR(v, 1.25f, qpred_tol(step, 1.25f));
    }
  }
}

TEST(DepthCodec, qpred_rejects_invalid_encode_step)
{
  const float one = 1.0f;
  std::vector<uint8_t> blob;
  EXPECT_THROW(
    depth_codec::encode_depth_quantized(&one, 1, 1, blob, 0.0f, 1), std::runtime_error);
  EXPECT_THROW(
    depth_codec::encode_depth_quantized(&one, 1, 1, blob, -0.001f, 1), std::runtime_error);
  EXPECT_THROW(
    depth_codec::encode_depth_quantized(
      &one, 1, 1, blob, std::numeric_limits<float>::quiet_NaN(), 1), std::runtime_error);
}

// Hand-crafted qpred blobs: payload = f32 step (uncompressed) | zstd(varints).
namespace
{
std::vector<uint8_t> make_qpred_blob(
  uint32_t w, uint32_t h, const std::vector<uint8_t> & step_bytes,
  const std::vector<uint8_t> & varints)
{
  std::vector<uint8_t> blob = make_blob_header("qpred", 1, w, h);
  blob.insert(blob.end(), step_bytes.begin(), step_bytes.end());
  const auto payload = zstd_compress_raw(varints);
  blob.insert(blob.end(), payload.begin(), payload.end());
  return blob;
}
}  // namespace

TEST(DepthCodec, qpred_rejects_malformed_blobs)
{
  const std::vector<uint8_t> good_step = {0x6F, 0x12, 0x83, 0x3A};  // 0.001f LE
  float out[4] = {};

  // Step = 0.0 -> rejected by read_header already (header must be sane).
  {
    const auto blob = make_qpred_blob(1, 1, {0, 0, 0, 0}, {0x00});
    EXPECT_THROW(depth_codec::read_header(blob.data(), blob.size()), std::runtime_error);
    EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), out), std::runtime_error);
  }
  // Step = NaN -> rejected.
  {
    const auto blob = make_qpred_blob(1, 1, {0x00, 0x00, 0xC0, 0x7F}, {0x00});
    EXPECT_THROW(depth_codec::read_header(blob.data(), blob.size()), std::runtime_error);
  }
  // Payload shorter than the step field -> rejected.
  {
    std::vector<uint8_t> blob = make_blob_header("qpred", 1, 1, 1);
    blob.insert(blob.end(), {0x6F, 0x12});
    EXPECT_THROW(depth_codec::read_header(blob.data(), blob.size()), std::runtime_error);
  }
  // Truncated varint stream (2x2 image, only 1 varint) -> rejected.
  {
    const auto blob = make_qpred_blob(2, 2, good_step, {0x02});
    EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), out), std::runtime_error);
  }
  // Trailing bytes after the varint stream -> rejected.
  {
    const auto blob = make_qpred_blob(1, 1, good_step, {0x02, 0x00});
    EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), out), std::runtime_error);
  }
  // Varint continuation past the 32-bit zigzag range -> rejected.
  {
    const auto blob = make_qpred_blob(
      1, 1, good_step, {0x80, 0x80, 0x80, 0x80, 0x80, 0x01});
    EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), out), std::runtime_error);
  }
  // Huge declared zstd content size -> rejected before allocation.
  {
    std::vector<uint8_t> blob = make_blob_header("qpred", 1, 1, 1);
    blob.insert(blob.end(), good_step.begin(), good_step.end());
    const auto payload = fake_huge_zstd_frame();
    blob.insert(blob.end(), payload.begin(), payload.end());
    EXPECT_THROW(depth_codec::decode_depth(blob.data(), blob.size(), out), std::runtime_error);
  }
}

// A valid depth whose v/step exceeds the representable code range must
// decode as NaN (invalid), never as a silently saturated value with
// unbounded error. Neighbouring on-grid pixels are unaffected.
TEST(DepthCodec, qpred_off_grid_depth_becomes_invalid)
{
  const float step = 1e-9f;  // absurdly fine: 10 m needs 1e10 codes >> 2^31
  std::vector<float> img = {10.0f, 1.0f, 2.0f, 0.5f};  // 10.0 is off-grid
  std::vector<uint8_t> blob;
  depth_codec::encode_depth_quantized(img.data(), 2, 2, blob, step, 1);
  std::vector<float> back(4);
  depth_codec::decode_depth(blob.data(), blob.size(), back.data());
  EXPECT_TRUE(std::isnan(back[0]));  // off-grid -> invalid, NOT ~2.147 m
  for (int i = 1; i < 4; ++i) {
    ASSERT_NEAR(back[i], img[i], qpred_tol(step, img[i])) << i;
  }
}
