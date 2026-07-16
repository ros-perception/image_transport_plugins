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

#include "zlib_wrapper.hpp"

namespace zlib_wrapper
{

std::size_t compressBound(std::size_t src_size)
{
  return ::compressBound(static_cast<uLong>(src_size));
}

std::size_t compress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size,
  int level)
{
  uLongf out_len = static_cast<uLongf>(dst_capacity);
  int ret = ::compress2(
    reinterpret_cast<Bytef *>(dst), &out_len,
    reinterpret_cast<const Bytef *>(src), static_cast<uLong>(src_size),
    level);
  if (ret != Z_OK) {
    return 0;
  }
  return static_cast<std::size_t>(out_len);
}

std::size_t decompress(
  uint8_t * dst, std::size_t dst_capacity,
  const uint8_t * src, std::size_t src_size)
{
  uLongf out_len = static_cast<uLongf>(dst_capacity);
  int ret = ::uncompress(
    reinterpret_cast<Bytef *>(dst), &out_len,
    reinterpret_cast<const Bytef *>(src), static_cast<uLong>(src_size));
  if (ret != Z_OK) {
    return 0;
  }
  return static_cast<std::size_t>(out_len);
}

}  // namespace zlib_wrapper
