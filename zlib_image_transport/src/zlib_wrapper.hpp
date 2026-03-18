// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#ifndef ZLIB_WRAPPER_HPP_
#define ZLIB_WRAPPER_HPP_

#include <zlib.h>

#include <cstddef>
#include <cstdint>

namespace zlib_wrapper
{

/// Returns the maximum compressed output size for src_size bytes of input.
std::size_t compressBound(std::size_t src_size);

/// Compress src into dst using zlib deflate at the given level (0–9).
/// dst must have at least compressBound(src_size) bytes.
/// Returns the actual compressed size, or 0 on error.
std::size_t compress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size,
  int level);

/// Decompress src into dst.
/// dst_capacity must be at least the original uncompressed size.
/// Returns the actual decompressed size, or 0 on error.
std::size_t decompress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size);

}  // namespace zlib_wrapper

#endif  // ZLIB_WRAPPER_HPP_
