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

#ifndef ZLIB_CPP_HPP_
#define ZLIB_CPP_HPP_

#include <zlib.h>

#include <list>
#include <memory>
#include <tuple>

namespace zlib
{

struct DataBlock
{
  uint8_t * ptr;
  std::size_t size;
};

std::shared_ptr<DataBlock> AllocateData(std::size_t size);
std::shared_ptr<DataBlock> ExpandDataList(const std::list<std::shared_ptr<DataBlock>> & data_list);

/// Compress processor.
class Comp
{
public:
  enum class Level
  {
    Default = -1,
    Min = 0,
    Level_1 = 1,
    Level_2 = 2,
    Level_3 = 3,
    Level_4 = 4,
    Level_5 = 5,
    Level_6 = 6,
    Level_7 = 7,
    Level_8 = 8,
    Max = 9
  };

public:
  /// Construct a compressor.
  explicit Comp(Level level = Level::Default, bool zlib_header = false);

  /// Destructor, will release z_stream.
  ~Comp();

  /// Returns true if compressor initialize successfully.
  bool IsSucc() const;

  /// Compress incoming buffer to DataBlock list.
  std::list<std::shared_ptr<DataBlock>> Process(
    const uint8_t * buffer, std::size_t size, bool last_block = false);

private:
  Level level_;
  z_stream zs_;
  bool init_ok_;
};

/// Decompress processor.
class Decomp
{
public:
  /// Construct a decompressor.
  Decomp();

  /// Destructor, will release z_stream.
  ~Decomp();

  /// Decompress incoming buffer to DataBlock list.
  std::list<std::shared_ptr<DataBlock>> Process(
    const std::shared_ptr<DataBlock> & compressed_data);

private:
  z_stream zs_;
  bool init_ok_;
};

}  // namespace zlib

#endif  // ZLIB_CPP_HPP_
