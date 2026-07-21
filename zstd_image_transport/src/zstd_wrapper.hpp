// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#ifndef ZSTD_WRAPPER_HPP_
#define ZSTD_WRAPPER_HPP_

#include <zstd.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace zstd_wrapper
{

/// Reusable ZSTD compression context.
/// Avoids per-frame context creation overhead.
class Compressor
{
public:
  Compressor();
  ~Compressor();

  // Non-copyable
  Compressor(const Compressor &) = delete;
  Compressor & operator=(const Compressor &) = delete;

  /// Compress src into dst. dst must be at least compressBound(srcSize) bytes.
  /// Returns actual compressed size, or 0 on error.
  std::size_t compress(
    uint8_t * dst, std::size_t dst_capacity,
    const uint8_t * src, std::size_t src_size,
    int level);

  /// Returns the maximum compressed size for src_size bytes of input.
  static std::size_t compressBound(std::size_t src_size);

private:
  ZSTD_CCtx * ctx_;
};

/// Reusable ZSTD decompression context.
class Decompressor
{
public:
  Decompressor();
  ~Decompressor();

  // Non-copyable
  Decompressor(const Decompressor &) = delete;
  Decompressor & operator=(const Decompressor &) = delete;

  /// Decompress src into dst.
  /// Returns actual decompressed size, or 0 on error.
  std::size_t decompress(
    uint8_t * dst, std::size_t dst_capacity,
    const uint8_t * src, std::size_t src_size);

  /// Returns the decompressed content size encoded in the zstd frame header.
  /// Returns 0 if the size is not stored in the frame or on error.
  static std::size_t getDecompressedSize(const uint8_t * src, std::size_t src_size);

private:
  ZSTD_DCtx * ctx_;
};

}  // namespace zstd_wrapper

#endif  // ZSTD_WRAPPER_HPP_
