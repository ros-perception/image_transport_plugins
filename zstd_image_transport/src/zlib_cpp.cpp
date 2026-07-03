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

#include "zlib_cpp.hpp"

namespace zlib
{

namespace
{
// Chunk size used to pull data out of zlib before appending to the output.
constexpr std::size_t MAX_CHUNK_SIZE = 1024;
constexpr int WINDOW_BITS = 15;
}  // namespace

Comp::Comp(Level level, bool zlib_header)
{
  int window_bits = WINDOW_BITS;
  if (zlib_header) {
    // Configure the compressor to write a simple zlib header and trailer
    // around the compressed data instead of a raw deflate stream.
    window_bits += 16;
  }
  init_ok_ = deflateInit2(
    &zs_, static_cast<int>(level), Z_DEFLATED, window_bits,
    8, Z_DEFAULT_STRATEGY) == Z_OK;
}

Comp::~Comp() {deflateEnd(&zs_);}

std::vector<std::uint8_t> Comp::Process(std::span<const std::uint8_t> input, bool last_block)
{
  std::vector<std::uint8_t> out;
  std::uint8_t chunk[MAX_CHUNK_SIZE];
  zs_.next_in = const_cast<Bytef *>(input.data());
  zs_.avail_in = static_cast<uInt>(input.size());
  do {
    zs_.avail_out = MAX_CHUNK_SIZE;
    zs_.next_out = chunk;
    deflate(&zs_, last_block ? Z_FINISH : Z_NO_FLUSH);
    out.insert(out.end(), chunk, chunk + (MAX_CHUNK_SIZE - zs_.avail_out));
  } while (zs_.avail_out == 0);
  return out;
}

Decomp::Decomp()
{
  // Enable zlib and gzip decoding with automatic header detection.
  init_ok_ = inflateInit2(&zs_, WINDOW_BITS + 32) == Z_OK;
}

Decomp::~Decomp() {inflateEnd(&zs_);}

std::vector<std::uint8_t> Decomp::Process(std::span<const std::uint8_t> input)
{
  std::vector<std::uint8_t> out;
  std::uint8_t chunk[MAX_CHUNK_SIZE];
  zs_.avail_in = static_cast<uInt>(input.size());
  zs_.next_in = const_cast<Bytef *>(input.data());
  do {
    zs_.avail_out = MAX_CHUNK_SIZE;
    zs_.next_out = chunk;
    int ret = inflate(&zs_, Z_NO_FLUSH);
    if (ret == Z_NEED_DICT || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
      // Incoming data is invalid or a critical error occurred.
      return out;
    }
    out.insert(out.end(), chunk, chunk + (MAX_CHUNK_SIZE - zs_.avail_out));
  } while (zs_.avail_out == 0);
  return out;
}

}  // namespace zlib
