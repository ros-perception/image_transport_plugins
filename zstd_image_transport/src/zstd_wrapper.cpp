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

#include "zstd_wrapper.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <iostream>

namespace zstd_wrapper
{

Compressor::Compressor()
: ctx_(ZSTD_createCCtx())
{
  if (!ctx_) {
    throw std::runtime_error("Failed to create ZSTD compression context");
  }
}

Compressor::~Compressor()
{
  ZSTD_freeCCtx(ctx_);
}

std::size_t Compressor::compress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size,
  int level)
{
  std::size_t result = ZSTD_compressCCtx(ctx_, dst, dst_capacity, src, src_size, level);
  if (ZSTD_isError(result)) {
    return 0;
  }
  return result;
}

std::size_t Compressor::compressBound(std::size_t src_size)
{
  return ZSTD_compressBound(src_size);
}

Decompressor::Decompressor()
: ctx_(ZSTD_createDCtx())
{
  if (!ctx_) {
    throw std::runtime_error("Failed to create ZSTD decompression context");
  }
}

Decompressor::~Decompressor()
{
  ZSTD_freeDCtx(ctx_);
}

std::size_t Decompressor::decompress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size)
{
  std::size_t result = ZSTD_decompressDCtx(ctx_, dst, dst_capacity, src, src_size);
  if (ZSTD_isError(result)) {
    return 0;
  }
  return result;
}

std::size_t Decompressor::getDecompressedSize(const uint8_t * src, std::size_t src_size)
{
  uint64_t size = ZSTD_getFrameContentSize(src, src_size);
  if (size == ZSTD_CONTENTSIZE_UNKNOWN || size == ZSTD_CONTENTSIZE_ERROR) {
    return 0;
  }
  return static_cast<std::size_t>(size);
}

}  // namespace zstd_wrapper
