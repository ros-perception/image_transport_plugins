// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "zlib_cpp.hpp"

#include <cstring>
#include <utility>

namespace zlib
{

// Block size block used to compress/uncompress the data
const int MAX_CHUNK_SIZE = 1024;

const int WINDOW_BITS = 15;

/// Allocate memory to DataBlock and assign to a shared_ptr object.
std::shared_ptr<DataBlock> AllocateData(std::size_t size)
{
  std::shared_ptr<DataBlock> data(new DataBlock, [](DataBlock * p) {
      delete[] p->ptr;
      delete p;
    });
  data->ptr = new uint8_t[size];
  data->size = size;
  return data;
}

std::shared_ptr<DataBlock> ExpandDataList(const std::list<std::shared_ptr<DataBlock>> & data_list)
{
  std::size_t total_size = 0;
  for (const std::shared_ptr<DataBlock> & this_data : data_list) {
    total_size += this_data->size;
  }
  std::shared_ptr<DataBlock> out_data = AllocateData(total_size);
  uint8_t * this_ptr = out_data->ptr;
  for (const std::shared_ptr<DataBlock> & this_data : data_list) {
    memcpy(this_ptr, this_data->ptr, this_data->size);
    this_ptr += this_data->size;
  }
  return out_data;
}

Comp::Comp(Level level, bool zlib_header)
: level_(level)
{
  memset(&zs_, 0, sizeof(zs_));
  int windowBits = WINDOW_BITS;
  if (zlib_header) {
    // Configurate the compressor to write a simple zlib header and trailer
    // around the compressed data instead of a zlib wrapper
    windowBits += 16;
  }
  int ret = deflateInit2(
    &zs_, static_cast<int>(level_), Z_DEFLATED, windowBits,
    8, Z_DEFAULT_STRATEGY);
  init_ok_ = ret == Z_OK;
}

Comp::~Comp() {deflateEnd(&zs_);}

bool Comp::IsSucc() const
{
  return init_ok_;
}

std::list<std::shared_ptr<DataBlock>> Comp::Process(
  const uint8_t * buffer, std::size_t size,
  bool last_block)
{
  std::list<std::shared_ptr<DataBlock>> out_data_list;
  // Prepare output buffer memory.
  uint8_t out_buffer[MAX_CHUNK_SIZE];
  zs_.next_in = reinterpret_cast<uint8_t *>(const_cast<uint8_t *>(buffer));
  zs_.avail_in = static_cast<uInt>(size);
  do {
    // Reset output buffer position and size.
    zs_.avail_out = MAX_CHUNK_SIZE;
    zs_.next_out = out_buffer;
    // Do compress.
    deflate(&zs_, last_block ? Z_FINISH : Z_NO_FLUSH);
    // Allocate output memory.
    std::size_t out_size = MAX_CHUNK_SIZE - zs_.avail_out;
    std::shared_ptr<DataBlock> out_data = AllocateData(out_size);
    // Copy and add to output data list.
    memcpy(out_data->ptr, out_buffer, out_size);
    out_data_list.push_back(std::move(out_data));
  } while (zs_.avail_out == 0);
  // Done.
  return out_data_list;
}

Decomp::Decomp()
{
  memset(&zs_, 0, sizeof(zs_));
  // Enable zlib and zlib decoding with automatic header detection
  int windowBits = WINDOW_BITS + 32;
  int ret = inflateInit2(&zs_, windowBits);
  init_ok_ = ret == Z_OK;
}

Decomp::~Decomp() {inflateEnd(&zs_);}

std::list<std::shared_ptr<DataBlock>> Decomp::Process(
  const std::shared_ptr<DataBlock> & compressed_data)
{
  std::list<std::shared_ptr<DataBlock>> out_data_list;
  uint8_t out_buffer[MAX_CHUNK_SIZE];
  // Incoming buffer.
  zs_.avail_in = static_cast<uInt>(compressed_data->size);
  zs_.next_in = compressed_data->ptr;
  int ret;
  do {
    // Prepare outcoming buffer and size.
    zs_.avail_out = MAX_CHUNK_SIZE;
    zs_.next_out = out_buffer;
    // Decompress data.
    ret = inflate(&zs_, Z_NO_FLUSH);
    switch (ret) {
      case Z_NEED_DICT:
        // Incoming data is invalid.
        return out_data_list;
      case Z_DATA_ERROR:
      case Z_MEM_ERROR:
        // Critical error.
        return out_data_list;
    }
    // Outcome size.
    std::size_t out_size = MAX_CHUNK_SIZE - zs_.avail_out;
    // Allocate outcome buffer.
    std::shared_ptr<DataBlock> out_data = AllocateData(out_size);
    memcpy(out_data->ptr, out_buffer, out_size);
    out_data_list.push_back(std::move(out_data));
  } while (zs_.avail_out == 0);
  return out_data_list;
}

}  // namespace zlib
