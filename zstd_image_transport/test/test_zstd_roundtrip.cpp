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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include <span>  // NOLINT(build/include_order) cpplint misclassifies <span>

#include "byte_order.hpp"
#include "zlib_cpp.hpp"

namespace
{

// Build a deterministic byte buffer of `n` bytes.
std::vector<std::uint8_t> makeBuffer(std::size_t n, std::uint8_t fill_pattern = 0)
{
  std::vector<std::uint8_t> b(n);
  for (std::size_t i = 0; i < n; ++i) {
    b[i] = fill_pattern ? fill_pattern : static_cast<std::uint8_t>((i * 31u + 7u) & 0xFFu);
  }
  return b;
}

std::vector<std::uint8_t> roundtrip(const std::vector<std::uint8_t> & input)
{
  zlib::Comp comp(zlib::Comp::Level::Level_6, true);
  EXPECT_TRUE(comp.IsSucc());
  const std::vector<std::uint8_t> compressed =
    comp.Process(std::span<const std::uint8_t>(input.data(), input.size()), true);

  zlib::Decomp decomp;
  return decomp.Process(std::span<const std::uint8_t>(compressed.data(), compressed.size()));
}

}  // namespace

// zlib is lossless: compress -> decompress must reproduce the input exactly,
// across a range of sizes (incl. the empty payload).
TEST(ZstdCodecRoundTrip, LosslessAcrossSizes)
{
  for (std::size_t n : {std::size_t{0}, std::size_t{1}, std::size_t{17}, std::size_t{1024},
      std::size_t{100000}})
  {
    const auto original = makeBuffer(n);
    const auto restored = roundtrip(original);
    EXPECT_EQ(restored, original) << "roundtrip mismatch for size " << n;
  }
}

// A highly repetitive payload must actually shrink (proves compression runs).
TEST(ZstdCodecRoundTrip, ShrinksCompressibleData)
{
  const auto original = makeBuffer(65536, 0xAB);
  zlib::Comp comp(zlib::Comp::Level::Level_6, true);
  const auto compressed =
    comp.Process(std::span<const std::uint8_t>(original.data(), original.size()), true);
  EXPECT_GT(compressed.size(), 0u);
  EXPECT_LT(compressed.size(), original.size());
  EXPECT_EQ(roundtrip(original), original);
}

// Invalid input must be rejected gracefully (no crash, empty output).
TEST(ZstdCodecRoundTrip, RejectsGarbage)
{
  const auto garbage = makeBuffer(64, 0x5A);
  zlib::Decomp decomp;
  const auto out = decomp.Process(std::span<const std::uint8_t>(garbage.data(), garbage.size()));
  EXPECT_TRUE(out.empty());
}

// The little-endian header helpers used by the (de)serializer must round-trip
// exactly, including values with the high bit set (the old shift code was UB).
TEST(ZstdByteOrder, StoreLoadRoundTrip)
{
  using zstd_image_transport::load_le;
  using zstd_image_transport::store_le;
  for (std::uint32_t v : {0u, 1u, 255u, 256u, 0x80000000u, 0xFFFFFFFFu, 0xDEADBEEFu}) {
    std::array<std::uint8_t, 4> b{};
    store_le(std::span<std::uint8_t, 4>(b), v);
    EXPECT_EQ(load_le<std::uint32_t>(std::span<const std::uint8_t, 4>(b)), v);
  }
  // Little-endian byte order: least-significant byte first.
  std::array<std::uint8_t, 4> b{};
  store_le(std::span<std::uint8_t, 4>(b), 0x11223344u);
  EXPECT_EQ(b[0], 0x44);
  EXPECT_EQ(b[3], 0x11);
}
