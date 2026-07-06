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

// Vendored "contrib" copy of the depth codec from
// https://github.com/facontidavide/depth_image_compression
// The blob format is identical. Methods implemented here:
//  - "dpred" (32FC1, lossless): per-image value dictionary sorted by float
//    total order, 2D MED prediction on the index plane, zigzag residual
//    byte planes, zstd entropy stage. Used when the image has <= 65536
//    distinct bit patterns.
//  - "fpred" (32FC1, lossless): dictionary-free variant for images that
//    overflow the 16-bit dictionary (typical full-precision float stereo
//    depth): MED prediction directly on the float total-order integers,
//    zigzag residuals in 4 byte planes, zstd. Automatic fallback of
//    encode_depth().
//  - "dpred16" (16UC1, lossless): the pixel value is already a small
//    monotone integer, i.e. its own sorted dictionary index, so the dpred
//    prediction core applies directly with no dictionary at all.
//  - "qpred" (32FC1, LOSSY): uniform quantization with a configurable step,
//    MED prediction on the quantization codes, LEB128-varint zigzag
//    residuals, zstd. Level count is unbounded (not limited to 16 bits).
//    Invalid inputs (NaN/inf/<= 0) decode as quiet NaN (REP 118).
// Lossless "dpred"/"fpred"/"dpred16" blobs are interchangeable with the
// standalone library in both directions ("store"/"zstd"/"bss"/"alp"/
// "alprd"/"dict" blobs from the standalone library are rejected here with
// "unknown method"). "qpred" originates in this plugin and is not decoded
// by standalone releases that predate it.
// See ALGORITHM.md in the source repository.

#ifndef DEPTH_CODEC_HPP_
#define DEPTH_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace depth_codec
{

enum class PixelFormat
{
  FLOAT32,  // 32FC1
  UINT16    // 16UC1
};

struct BlobHeader
{
  uint32_t width;
  uint32_t height;
  PixelFormat format;
  // 0.0 for the lossless methods; the uniform quantization step in meters
  // for "qpred" blobs (stored uncompressed in the blob, so this is
  // available without decompressing anything). Together with
  // width/height/format this is everything needed to decode the blob and
  // interpret the result.
  float quantization_step;
};

constexpr size_t bytes_per_pixel(PixelFormat format)
{
  return format == PixelFormat::FLOAT32 ? 4 : 2;
}

/// Losslessly compress width*height float32 (32FC1) depth pixels into `out`
/// (replacing its contents; existing capacity is reused, so a caller that
/// keeps the vector alive across frames pays no steady-state output
/// allocations). Bit-exact round trip, NaN payloads included. Emits a
/// "dpred" blob, or an "fpred" blob when the image has more than 65536
/// distinct bit patterns. zstd_level is clamped to [1, 3]. The blob is
/// self-describing.
void encode_depth(
  const float * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, int zstd_level = 1);

/// LOSSY variant ("qpred" blob): quantizes valid depth to a uniform grid of
/// `step` meters before compressing, reconstructing within +/- step/2 (up
/// to float32 rounding of the reconstructed value, i.e. a few ULPs).
/// Invalid pixels (NaN, +/-inf, <= 0) are preserved as invalid and decode
/// as quiet NaN (REP 118); so does any depth too large for the grid
/// (v/step beyond ~2^31 -- with the default 0.1 mm step that is ~214 km),
/// rather than silently saturating with unbounded error. The quantized
/// level count is not limited to 16 bits, so arbitrarily fine steps and
/// long ranges are supported. Throws std::runtime_error unless step is
/// positive and finite.
void encode_depth_quantized(
  const float * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, float step, int zstd_level = 1);

/// Compress width*height uint16 (16UC1) depth pixels into `out`.
void encode_depth16(
  const uint16_t * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, int zstd_level = 1);

/// Parse the self-describing blob header (cheap, no decompression).
/// Use it to size the output buffer and select the decode function.
/// Throws std::runtime_error on malformed or unknown blobs.
BlobHeader read_header(const uint8_t * blob, size_t size);

/// Decode a FLOAT32 blob into a caller-provided buffer of exactly
/// width*height floats (see read_header). Bit-exact round trip; no output
/// allocation or copy. Throws std::runtime_error on malformed input or a
/// pixel-format mismatch.
void decode_depth(const uint8_t * blob, size_t size, float * out);

/// Decode a UINT16 blob into a caller-provided buffer of width*height values.
void decode_depth16(const uint8_t * blob, size_t size, uint16_t * out);

}  // namespace depth_codec

#endif  // DEPTH_CODEC_HPP_
