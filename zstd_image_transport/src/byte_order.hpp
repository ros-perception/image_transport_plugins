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

#ifndef BYTE_ORDER_HPP_
#define BYTE_ORDER_HPP_

#include <cstddef>
#include <cstdint>

#include <concepts>  // NOLINT(build/include_order) cpplint misclassifies C++20 headers
#include <span>  // NOLINT(build/include_order) cpplint misclassifies <span>

namespace zstd_image_transport
{

/// Serialize `value` into `out` as little-endian bytes.
///
/// The fixed-extent span makes the width a compile-time contract: passing a
/// span whose size differs from sizeof(T) is a compile error, not a runtime bug.
template<std::unsigned_integral T>
constexpr void store_le(std::span<std::uint8_t, sizeof(T)> out, T value) noexcept
{
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

/// Deserialize a little-endian `T` from `in`.
template<std::unsigned_integral T>
constexpr T load_le(std::span<const std::uint8_t, sizeof(T)> in) noexcept
{
  T value = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    value = static_cast<T>(value | (static_cast<T>(in[i]) << (8 * i)));
  }
  return value;
}

}  // namespace zstd_image_transport

#endif  // BYTE_ORDER_HPP_
