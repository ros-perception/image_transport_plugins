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

#include "depth_codec.hpp"

#include <zstd.h>

// Runtime AVX2 dispatch for the dictionary probe: released binaries build
// with portable flags, so the SIMD path is selected per-CPU at load time
// (GCC/Clang on x86-64; other targets use the scalar probe).
#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))
#define DEPTH_CODEC_AVX2_DISPATCH 1
#include <immintrin.h>
#endif

#include <algorithm>
#include <bit>  // NOLINT(build/include_order) -- cpplint predates C++20 headers
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace depth_codec
{
namespace
{

[[noreturn]] void fail(const char * what)
{
  throw std::runtime_error(what);
}

// ---- zstd stage with per-thread context reuse ------------------------------
struct ZstdCtx
{
  ZSTD_CCtx * c = ZSTD_createCCtx();
  ZSTD_DCtx * d = ZSTD_createDCtx();
  ~ZstdCtx()
  {
    ZSTD_freeCCtx(c);
    ZSTD_freeDCtx(d);
  }
};

ZstdCtx & tl_zstd()
{
  static thread_local ZstdCtx ctx;
  return ctx;
}

#if defined(DEPTH_CODEC_AVX2_DISPATCH)
// Single dispatch policy for every SIMD kernel in this file.
inline bool cpu_has_avx2()
{
  static const bool v = __builtin_cpu_supports("avx2");
  return v;
}
#endif

// Compress src and append the zstd frame to `out`. Compresses into a
// reused thread-local buffer first: `out` is typically a fresh outgoing
// message, and sizing it to ZSTD_compressBound would cost a multi-MB
// alloc + zero-fill per frame and leave the published message holding
// bound-sized capacity for its whole lifetime.
void zstd_append(std::vector<uint8_t> & out, const uint8_t * src, size_t n, int level)
{
  static thread_local std::vector<uint8_t> comp;
  const size_t bound = ZSTD_compressBound(n);
  comp.resize(bound);
  const size_t k = ZSTD_compressCCtx(tl_zstd().c, comp.data(), bound, src, n, level);
  if (ZSTD_isError(k)) {
    fail(ZSTD_getErrorName(k));
  }
  out.insert(out.end(), comp.data(), comp.data() + k);
}

// Decompress a zstd frame into `out` (capacity reused across calls -- see
// tl_decode_scratch()). `expected_size` bounds the frame's self-reported
// content size *before* anything is allocated: a hostile/corrupt frame can
// claim an arbitrary ZSTD_getFrameContentSize (e.g. a few compressed bytes
// "inflating" to many GiB), so the claim is validated against a size the
// caller already trusts (derived from the image dimensions) instead of
// being allocated blindly.
//   exact_size = true:  content size must equal `expected_size` exactly
//                       (the caller has no further variable-size header to
//                       parse, e.g. the fixed-layout 16UC1 payload).
//   exact_size = false: `expected_size` is only an upper bound; the caller
//                       still validates the actual size afterwards once it
//                       has parsed enough of the payload to know the exact
//                       expected value (e.g. the dpred dictionary size,
//                       which is itself part of the compressed content).
void zstd_unpack(
  const uint8_t * comp, size_t comp_size, size_t expected_size, bool exact_size,
  std::vector<uint8_t> & out)
{
  const unsigned long long sz = ZSTD_getFrameContentSize(comp, comp_size);  // NOLINT
  if (sz == ZSTD_CONTENTSIZE_ERROR || sz == ZSTD_CONTENTSIZE_UNKNOWN) {
    fail("bad zstd frame");
  }
  const bool size_ok = exact_size ?
    (sz == expected_size) : (sz <= static_cast<unsigned long long>(expected_size));  // NOLINT
  if (!size_ok) {
    fail("zstd_unpack: frame content size out of the expected range");
  }
  out.resize(static_cast<size_t>(sz));
  const size_t k = ZSTD_decompressDCtx(tl_zstd().d, out.data(), out.size(), comp, comp_size);
  if (ZSTD_isError(k)) {
    fail(ZSTD_getErrorName(k));
  }
  if (k != out.size()) {
    fail("zstd_unpack: size mismatch");
  }
}

// ---- decode-side thread-local scratch --------------------------------------
// Steady-state decoding (stable image size) should not allocate: `plain`
// (the zstd_unpack destination) and `idx` (the dpred index plane) are
// resized, never freed, across calls. Neither buffer escapes decode_depth /
// decode_depth16 -- both write their final result into the caller-owned
// output pointer -- so reuse across frames/threads-of-one is safe.
struct DecodeScratch
{
  std::vector<uint8_t> plain;
  std::vector<uint16_t> idx;
  std::vector<uint32_t> words32;    // fpred ord plane / qpred code plane
  std::vector<uint32_t> residuals;  // qpred zigzag residual plane
};

DecodeScratch & tl_decode_scratch()
{
  static thread_local DecodeScratch scratch;
  return scratch;
}

// ---- little-endian helpers --------------------------------------------------
inline void put_u32(std::vector<uint8_t> & v, uint32_t x)
{
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>(x >> (8 * i)));
  }
}

inline uint32_t get_u32(const uint8_t * p)
{
  uint32_t x = 0;
  for (int i = 0; i < 4; ++i) {
    x |= static_cast<uint32_t>(p[i]) << (8 * i);
  }
  return x;
}

// Total-order-preserving bijection: unsigned comparison of the mapped word
// equals IEEE-754 total order of the float (negatives fixed up, NaN at the
// top).
inline uint32_t float_to_ord(uint32_t w)
{
  return (w >> 31) ? ~w : (w | 0x80000000u);
}

inline uint32_t ord_to_float(uint32_t m)
{
  return (m >> 31) ? (m ^ 0x80000000u) : ~m;
}

// JPEG-LS / LOCO-I median-edge-detector predictor, branchless clamp form.
inline int32_t med_predict(int32_t a, int32_t b, int32_t c)
{
  const int32_t mn = std::min(a, b);
  const int32_t mx = std::max(a, b);
  return std::clamp(a + b - c, mn, mx);
}

// uint32 variant for the fpred/qpred paths, identical to the standalone
// library's med_predict: min/max in the unsigned domain, and the gradient
// case wraps mod 2^32 (the decoder mirrors the wrap, so it is bijective).
inline uint32_t med_predict_u32(uint32_t a /*left*/, uint32_t b /*up*/, uint32_t c /*upleft*/)
{
  const uint32_t mx = std::max(a, b), mn = std::min(a, b);
  if (c >= mx) {return mn;}
  if (c <= mn) {return mx;}
  return a + b - c;
}

inline uint32_t zigzag32(uint32_t d)  // d = wrapped (mod 2^32) difference
{
  const int32_t s = static_cast<int32_t>(d);
  return (static_cast<uint32_t>(s) << 1) ^ static_cast<uint32_t>(s >> 31);
}

inline uint32_t unzigzag32(uint32_t z)
{
  return (z >> 1) ^ (~(z & 1) + 1);
}

// Raster-order MED predictor with the standalone library's edge rules
// (row 0 predicts from the left, column 0 from above). Shared by fpred
// encode and decode, which must agree bit-for-bit.
inline uint32_t raster_pred_u32(const uint32_t * v, size_t i, uint32_t x, uint32_t y, uint32_t w)
{
  if (y == 0) {
    return x ? v[i - 1] : 0;
  }
  if (x == 0) {
    return v[i - w];
  }
  return med_predict_u32(v[i - 1], v[i - w], v[i - w - 1]);
}

// ---- shared prediction core --------------------------------------------------
// MED-predict each uint16 in raster order from its left/up/up-left
// neighbours, zigzag the mod-2^16 residual (bijective for any jump size)
// and split it into a low-byte and a high-byte plane. For smooth depth the
// planes are mostly zeros, which the zstd stage then collapses.
void predict_pack(const uint16_t * vals, uint32_t w, uint32_t h, uint8_t * lo, uint8_t * hi)
{
  const auto emit = [&](size_t i, int32_t pred) {
      const int16_t s = static_cast<int16_t>(static_cast<uint16_t>(vals[i] - pred));
      const uint16_t z = static_cast<uint16_t>((s << 1) ^ (s >> 15));
      lo[i] = static_cast<uint8_t>(z);
      hi[i] = static_cast<uint8_t>(z >> 8);
    };
  if (w == 0 || h == 0) {
    return;
  }
  emit(0, 0);
  for (uint32_t x = 1; x < w; ++x) {
    emit(x, vals[x - 1]);
  }
  for (uint32_t y = 1; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    emit(row, vals[row - w]);
    const uint16_t * up = vals + row - w;
    const uint16_t * cur = vals + row;
    for (uint32_t x = 1; x < w; ++x) {
      emit(row + x, med_predict(cur[x - 1], up[x], up[x - 1]));
    }
  }
}

// Inverse of predict_pack. Reconstruction is serial along a row (each
// prediction needs the value just decoded), but row y+1 at column c only
// needs row y up to column c: processing R rows along a skewed diagonal
// ("wavefront") therefore runs R independent dependency chains that the
// out-of-order core overlaps. Pure decoder-side optimization: the format
// and the results are identical to the serial scan.
void predict_unpack(const uint8_t * lo, const uint8_t * hi, uint32_t w, uint32_t h, uint16_t * vals)
{
  const auto unstep = [&](size_t i, int32_t pred) -> uint16_t {
      const uint16_t z = static_cast<uint16_t>(lo[i] | (hi[i] << 8));
      const uint16_t r = static_cast<uint16_t>((z >> 1) ^ (~(z & 1) + 1));
      const uint16_t k = static_cast<uint16_t>(pred + r);
      vals[i] = k;
      return k;
    };
  if (w == 0 || h == 0) {
    return;
  }
  // First row: pure left-prediction chain.
  unstep(0, 0);
  for (uint32_t x = 1; x < w; ++x) {
    unstep(x, vals[x - 1]);
  }

  constexpr uint32_t R = 4;  // interleaved rows = parallel dependency chains
  uint32_t y = 1;
  if (w >= 2 * R) {
    for (; y + R <= h; y += R) {
      uint16_t left[R] = {};
      // Ramp-up: row y+r starts one diagonal step after row y+r-1, which
      // keeps the in-strip dependency satisfied (row r reads row r-1 one
      // step behind).
      for (uint32_t t = 0; t < R; ++t) {
        for (uint32_t r = 0; r <= t; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint16_t * up = vals + row - w;
          left[r] = (c == 0) ?
            unstep(row, up[0]) :
            unstep(row + c, med_predict(left[r], up[c], up[c - 1]));
        }
      }
      // Steady state: all R chains active, no bounds checks.
      for (uint32_t t = R; t < w; ++t) {
        for (uint32_t r = 0; r < R; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint16_t * up = vals + row - w;
          left[r] = unstep(row + c, med_predict(left[r], up[c], up[c - 1]));
        }
      }
      // Drain: finish the trailing columns of the lower rows.
      for (uint32_t t = w; t < w + R - 1; ++t) {
        for (uint32_t r = t - w + 1; r < R; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint16_t * up = vals + row - w;
          left[r] = unstep(row + c, med_predict(left[r], up[c], up[c - 1]));
        }
      }
    }
  }
  // Remaining rows (strip remainder, or narrow images): serial scan.
  for (; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    int32_t left = unstep(row, vals[row - w]);
    const uint16_t * up = vals + row - w;
    for (uint32_t x = 1; x < w; ++x) {
      left = unstep(row + x, med_predict(left, up[x], up[x - 1]));
    }
  }
}

// ---- per-image value dictionary, bucketized + SIMD-probed -------------------
// Maps each 32-bit pattern to a first-seen id. 8 keys per cache-line bucket,
// all compared at once; a per-bucket count masks stale slots. The found
// branch is ~99% predictable (only first occurrences miss), which keeps the
// pipeline from flushing. Returns false if > 65536 distinct patterns.
struct alignas(64) DictBucket
{
  uint32_t keys[8];
  uint16_t vals[8];
  uint16_t cnt;
  uint16_t pad[7];
};
static_assert(sizeof(DictBucket) == 64, "bucket must be one cache line");

constexpr size_t kDictBuckets = 1u << 14;  // 16384 buckets x 8 slots, > 2x max load

// Reused across frames, but every bucket's `cnt` must return to 0: memset
// the reused block (cheaper than a fresh alloc + first-touch page faults).
void reset_dict_table(std::vector<DictBucket> & table)
{
  table.resize(kDictBuckets);
  std::memset(table.data(), 0, table.size() * sizeof(DictBucket));
}

bool build_value_dict_scalar(
  const uint32_t * words, size_t n, std::vector<DictBucket> & table,
  std::vector<uint32_t> & entries, std::vector<uint16_t> & idx)
{
  constexpr size_t nb = kDictBuckets;
  constexpr size_t max_dict = 1u << 16;
  reset_dict_table(table);
  entries.clear();
  entries.reserve(1u << 12);
  idx.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const uint32_t k32 = words[i];
    size_t b = (k32 * 2654435761u) >> 18;  // top 14 bits
    uint32_t id;
    for (;; ) {
      DictBucket & bucket = table[b];
      uint32_t m = 0;
      for (unsigned k = 0; k < bucket.cnt; ++k) {
        if (bucket.keys[k] == k32) {
          m = 1u << k;
          break;
        }
      }
      if (m) {
        id = bucket.vals[std::countr_zero(m)];
        break;
      }
      if (bucket.cnt < 8) {
        if (entries.size() >= max_dict) {
          return false;
        }
        id = static_cast<uint32_t>(entries.size());
        bucket.keys[bucket.cnt] = k32;
        bucket.vals[bucket.cnt] = static_cast<uint16_t>(id);
        ++bucket.cnt;
        entries.push_back(k32);
        break;
      }
      b = (b + 1) & (nb - 1);  // bucket full: spill to the next one
    }
    idx[i] = static_cast<uint16_t>(id);
  }
  return true;
}

#if defined(DEPTH_CODEC_AVX2_DISPATCH)
// Verbatim copy of build_value_dict_scalar with the probe replaced by one
// 8-wide SIMD compare. Compiled with the avx2 target attribute so portable
// (non -mavx2) builds still contain it and can select it at runtime; a
// shared inline body cannot carry a per-caller target attribute.
__attribute__((target("avx2"))) bool build_value_dict_avx2(
  const uint32_t * words, size_t n, std::vector<DictBucket> & table,
  std::vector<uint32_t> & entries, std::vector<uint16_t> & idx)
{
  constexpr size_t nb = kDictBuckets;
  constexpr size_t max_dict = 1u << 16;
  reset_dict_table(table);
  entries.clear();
  entries.reserve(1u << 12);
  idx.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const uint32_t k32 = words[i];
    size_t b = (k32 * 2654435761u) >> 18;
    uint32_t id;
    for (;; ) {
      DictBucket & bucket = table[b];
      const __m256i vk = _mm256_set1_epi32(static_cast<int32_t>(k32));
      const __m256i keys =
        _mm256_loadu_si256(reinterpret_cast<const __m256i *>(bucket.keys));
      uint32_t m = static_cast<uint32_t>(
        _mm256_movemask_ps(_mm256_castsi256_ps(_mm256_cmpeq_epi32(keys, vk))));
      m &= (1u << bucket.cnt) - 1;
      if (m) {
        id = bucket.vals[std::countr_zero(m)];
        break;
      }
      if (bucket.cnt < 8) {
        if (entries.size() >= max_dict) {
          return false;
        }
        id = static_cast<uint32_t>(entries.size());
        bucket.keys[bucket.cnt] = k32;
        bucket.vals[bucket.cnt] = static_cast<uint16_t>(id);
        ++bucket.cnt;
        entries.push_back(k32);
        break;
      }
      b = (b + 1) & (nb - 1);
    }
    idx[i] = static_cast<uint16_t>(id);
  }
  return true;
}
#endif  // DEPTH_CODEC_AVX2_DISPATCH

bool build_value_dict(
  const uint32_t * words, size_t n, std::vector<DictBucket> & table,
  std::vector<uint32_t> & entries, std::vector<uint16_t> & idx)
{
#if defined(DEPTH_CODEC_AVX2_DISPATCH)
  if (cpu_has_avx2()) {
    return build_value_dict_avx2(words, n, table, entries, idx);
  }
#endif
  return build_value_dict_scalar(words, n, table, entries, idx);
}

// ---- encode-side thread-local scratch --------------------------------------
// Same reuse/lifetime rationale as DecodeScratch; nothing here escapes the
// encode functions (the result is copied out via zstd_append).
struct EncodeScratch
{
  std::vector<DictBucket> dict_table;
  std::vector<uint32_t> entries;
  std::vector<uint16_t> idx;
  std::vector<uint64_t> keys_a, keys_b;
  std::vector<uint32_t> radix_hist;
  std::vector<uint16_t> rank;
  std::vector<uint32_t> sorted_entries;
  std::vector<uint8_t> plain;
  std::vector<uint32_t> words32;    // fpred ord plane / qpred code plane
  std::vector<uint32_t> residuals;  // qpred zigzag residual plane
};

EncodeScratch & tl_encode_scratch()
{
  static thread_local EncodeScratch scratch;
  return scratch;
}

// ---- dpred payload (32FC1) ---------------------------------------------------
// A single zstd frame whose decompressed content is (little-endian):
//   u8 mode           0 = legacy fallback (raw words; decode-only, the
//                     encoder now emits an "fpred" blob instead when the
//                     dictionary overflows), 1 = dictionary
//   u8 idx_bytes      always 2
//   u32 dict_size     ds <= 65536
//   u32 x ds          dictionary, sorted by float total order
//   u8  x n           low bytes of zigzag residuals
//   u8  x n           high bytes of zigzag residuals
//
// Precondition: build_value_dict succeeded and left the dictionary in
// scratch.entries and the first-seen index plane in scratch.idx (the caller
// runs it first, because the outcome decides the blob method name).
void dpred_encode_with_dict(
  uint32_t w, uint32_t h, int level, EncodeScratch & scratch, std::vector<uint8_t> & out)
{
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint32_t> & entries = scratch.entries;
  std::vector<uint16_t> & idx = scratch.idx;
  std::vector<uint8_t> & plain = scratch.plain;

  // Sort the dictionary by float total order and remap indices so that
  // index distance ~ depth distance (what makes MED residuals small).
  // LSD radix sort (two 16-bit digits) over key = ord<<16 | original_index.
  const size_t ds = entries.size();
  std::vector<uint64_t> & keys_a = scratch.keys_a;
  std::vector<uint64_t> & keys_b = scratch.keys_b;
  keys_a.resize(ds);
  keys_b.resize(ds);
  for (size_t k = 0; k < ds; ++k) {
    keys_a[k] = (static_cast<uint64_t>(float_to_ord(entries[k])) << 16) | k;
  }
  {
    std::vector<uint32_t> & hist = scratch.radix_hist;
    hist.resize(1u << 16);
    uint64_t * src = keys_a.data();
    uint64_t * dst = keys_b.data();
    for (const int shift : {16, 32}) {
      std::fill(hist.begin(), hist.end(), 0);
      for (size_t k = 0; k < ds; ++k) {
        ++hist[(src[k] >> shift) & 0xFFFF];
      }
      uint32_t sum = 0;
      for (uint32_t & slot : hist) {
        const uint32_t c = slot;
        slot = sum;
        sum += c;
      }
      for (size_t k = 0; k < ds; ++k) {
        dst[hist[(src[k] >> shift) & 0xFFFF]++] = src[k];
      }
      std::swap(src, dst);
    }
  }
  std::vector<uint16_t> & rank = scratch.rank;
  std::vector<uint32_t> & sorted_entries = scratch.sorted_entries;
  rank.resize(ds);
  sorted_entries.resize(ds);
  for (size_t k = 0; k < ds; ++k) {
    const uint32_t orig = static_cast<uint32_t>(keys_a[k] & 0xFFFF);
    rank[orig] = static_cast<uint16_t>(k);
    sorted_entries[k] = entries[orig];
  }
  for (size_t i = 0; i < n; ++i) {
    idx[i] = rank[idx[i]];
  }

  plain.resize(6 + ds * 4 + n * 2);
  plain[0] = 1;
  plain[1] = 2;  // residuals are always 2 bytes (split into two planes)
  static_assert(std::endian::native == std::endian::little, "format is little-endian");
  const uint32_t ds32 = static_cast<uint32_t>(ds);
  std::memcpy(plain.data() + 2, &ds32, 4);
  // ds == 0 (empty image): sorted_entries.data() may be null, and memcpy
  // with a null src is UB even for 0 bytes.
  if (ds > 0) {
    std::memcpy(plain.data() + 6, sorted_entries.data(), ds * 4);
  }

  uint8_t * lo = plain.data() + 6 + ds * 4;
  predict_pack(idx.data(), w, h, lo, lo + n);
  zstd_append(out, plain.data(), plain.size(), level);
}

void dpred_decode(const uint8_t * comp, size_t comp_size, float * out, uint32_t w, uint32_t h)
{
  const size_t n = static_cast<size_t>(w) * h;
  DecodeScratch & scratch = tl_decode_scratch();
  std::vector<uint8_t> & plain = scratch.plain;

  // The exact decompressed size depends on the payload's own mode byte and
  // dict_size field, so pre-decompression only an n-derived upper bound can
  // be enforced (mode 0: 1 + 4n; mode 1: <= 6 + 4*65536 + 2n); the exact
  // size is validated below, after parsing.
  const size_t max_plain = std::max<size_t>(1 + n * 4, 6 + (size_t{1} << 16) * 4 + n * 2);
  zstd_unpack(comp, comp_size, max_plain, /*exact_size=*/false, plain);
  if (plain.empty()) {
    fail("dpred_decode: empty payload");
  }
  uint32_t * words = reinterpret_cast<uint32_t *>(out);
  const uint8_t mode = plain[0];
  if (mode == 0) {
    if (plain.size() != 1 + n * 4) {
      fail("dpred_decode: size mismatch");
    }
    // n == 0: `words` may be null, and memcpy with a null dst is UB even
    // for 0 bytes.
    if (n > 0) {
      std::memcpy(words, plain.data() + 1, n * 4);
    }
    return;
  }
  if (mode != 1) {
    fail("dpred_decode: invalid mode byte");
  }
  if (plain.size() < 6) {
    fail("dpred_decode: truncated header");
  }
  if (plain[1] != 2) {
    fail("dpred_decode: invalid idx_bytes (expected 2)");
  }
  const size_t ds = get_u32(&plain[2]);
  if (plain.size() != 6 + ds * 4 + n * 2) {
    fail("dpred_decode: size mismatch");
  }
  const uint8_t * dict = plain.data() + 6;  // read in place (get_u32 handles alignment)
  const uint8_t * lo = plain.data() + 6 + ds * 4;

  std::vector<uint16_t> & idx = scratch.idx;
  idx.resize(n);
  predict_unpack(lo, lo + n, w, h, idx.data());
  for (size_t i = 0; i < n; ++i) {
    if (idx[i] >= ds) {
      fail("dpred_decode: bad index");
    }
    words[i] = get_u32(dict + 4 * static_cast<size_t>(idx[i]));
  }
}

// ---- fpred payload (32FC1) ---------------------------------------------------
// Dictionary-free fallback for images whose distinct-value count overflows
// the 16-bit dictionary (typical for full-precision float stereo depth,
// where most pixels carry a unique mantissa pattern). Byte-identical to the
// standalone library's "fpred" method: map every word to its total-order
// integer, MED-predict from the left/up/up-left neighbours, zigzag the
// mod-2^32 residual and split it into 4 byte planes for zstd. Smooth depth
// makes residuals tiny, so the high planes collapse to near-zero runs.
void fpred_encode(
  const float * data, uint32_t w, uint32_t h, int level,
  EncodeScratch & scratch, std::vector<uint8_t> & out)
{
  const size_t n = static_cast<size_t>(w) * h;
  const uint32_t * words = reinterpret_cast<const uint32_t *>(data);
  std::vector<uint32_t> & ord = scratch.words32;
  ord.resize(n);
  for (size_t i = 0; i < n; ++i) {
    ord[i] = float_to_ord(words[i]);
  }

  std::vector<uint8_t> & planes = scratch.plain;
  planes.resize(n * 4);
  for (uint32_t y = 0; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    for (uint32_t x = 0; x < w; ++x) {
      const size_t i = row + x;
      const uint32_t pred = raster_pred_u32(ord.data(), i, x, y, w);
      const uint32_t z = zigzag32(ord[i] - pred);
      planes[i] = static_cast<uint8_t>(z);
      planes[n + i] = static_cast<uint8_t>(z >> 8);
      planes[2 * n + i] = static_cast<uint8_t>(z >> 16);
      planes[3 * n + i] = static_cast<uint8_t>(z >> 24);
    }
  }
  zstd_append(out, planes.data(), planes.size(), level);
}

void fpred_decode(const uint8_t * comp, size_t comp_size, float * out, uint32_t w, uint32_t h)
{
  const size_t n = static_cast<size_t>(w) * h;
  DecodeScratch & scratch = tl_decode_scratch();
  std::vector<uint8_t> & planes = scratch.plain;
  // The fpred payload has no variable-size header: exactly 4 byte planes.
  zstd_unpack(comp, comp_size, n * 4, /*exact_size=*/true, planes);
  uint32_t * words = reinterpret_cast<uint32_t *>(out);
  std::vector<uint32_t> & ord = scratch.words32;
  ord.resize(n);
  for (uint32_t y = 0; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    for (uint32_t x = 0; x < w; ++x) {
      const size_t i = row + x;
      const uint32_t pred = raster_pred_u32(ord.data(), i, x, y, w);
      const uint32_t z = planes[i] | (planes[n + i] << 8) |
        (static_cast<uint32_t>(planes[2 * n + i]) << 16) |
        (static_cast<uint32_t>(planes[3 * n + i]) << 24);
      ord[i] = pred + unzigzag32(z);
      words[i] = ord_to_float(ord[i]);
    }
  }
}

// ---- qpred payload (32FC1, lossy) ---------------------------------------------
// Configurable uniform quantization. Payload layout (little-endian):
//   f32 step          quantization step in meters (> 0), UNCOMPRESSED so
//                     that read_header() can report it without touching the
//                     zstd frame -- together with the blob header this makes
//                     the message fully self-describing
//   zstd frame of:
//     LEB128 varint x n   zigzag(code - MED(neighbours)), raster order
// with code 0 reserved for invalid pixels (NaN/inf/<= 0; decoded back as
// quiet NaN per REP 118) and code = round(depth/step) + 1 otherwise. The
// level count is unbounded (fine steps and long ranges exceed 16 bits, e.g.
// 10 m at 0.1 mm = 100k levels), which is why residuals are varint-coded:
// after MED prediction almost all of them fit one byte regardless of the
// code width, and depth discontinuities cost 2-3 bytes instead of forcing a
// fixed width on the whole plane.
constexpr size_t kMaxVarintBytes = 5;  // 32-bit zigzag -> <= 35 payload bits

// Quantize one depth value. Float ops in this exact sequence (mul, compare,
// add 0.5, truncate) so the scalar and AVX2 paths produce bit-identical
// code planes; round-half-up via +0.5/truncate instead of lrint/cvtps makes
// the result independent of the ambient FPU rounding mode as well.
//
// A depth too large to sit on the quantization grid (v/step beyond ~2^31,
// e.g. a garbage sample, or a far pixel under an absurdly fine step) is
// treated as INVALID (code 0 -> NaN), like NaN/inf/<= 0: every pixel the
// decoder reports as a number is genuinely within +/- step/2 of the input,
// and unrepresentable ones fail loudly as NaN instead of silently
// saturating with an unbounded error. With the default 0.1 mm step the
// cutoff is ~214 km, unreachable for any real sensor.
constexpr float kMaxQuantFloat = 2147483520.0f;  // largest float < 2^31

inline uint32_t quantize_code(float v, float inv_step)
{
  const float q = v * inv_step;
  // Single validity test, false for NaN/inf/<= 0 inputs, float overflow of
  // the multiply, and codes past the grid.
  if (!(v > 0.0f) || !(q <= kMaxQuantFloat)) {
    return 0;
  }
  return static_cast<uint32_t>(q + 0.5f) + 1;
}

void qpred_quantize_scalar(const float * data, size_t n, float inv_step, uint32_t * code)
{
  for (size_t i = 0; i < n; ++i) {
    code[i] = quantize_code(data[i], inv_step);
  }
}

// Zigzag residuals against the branch-form MED predictor. The encoder
// predicts from the ORIGINAL code plane (the decoder reconstructs the very
// same values), so unlike the decoder this has no serial dependency and the
// rows vectorize directly. The predictor must match med_predict_u32
// bit-for-bit or the decoder diverges.
void qpred_residuals_scalar(const uint32_t * code, uint32_t w, uint32_t h, uint32_t * zres)
{
  zres[0] = zigzag32(code[0]);
  for (uint32_t x = 1; x < w; ++x) {
    zres[x] = zigzag32(code[x] - code[x - 1]);
  }
  for (uint32_t y = 1; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    zres[row] = zigzag32(code[row] - code[row - w]);
    for (uint32_t x = 1; x < w; ++x) {
      const size_t i = row + x;
      zres[i] = zigzag32(code[i] - med_predict_u32(code[i - 1], code[i - w], code[i - w - 1]));
    }
  }
}

#if defined(DEPTH_CODEC_AVX2_DISPATCH)
__attribute__((target("avx2"))) void qpred_quantize_avx2(
  const float * data, size_t n, float inv_step, uint32_t * code)
{
  const __m256 vinv = _mm256_set1_ps(inv_step);
  const __m256 vzero = _mm256_setzero_ps();
  const __m256 vqmax = _mm256_set1_ps(kMaxQuantFloat);
  const __m256 vhalf = _mm256_set1_ps(0.5f);
  const __m256i vone = _mm256_set1_epi32(1);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const __m256 v = _mm256_loadu_ps(data + i);
    const __m256 q = _mm256_mul_ps(v, vinv);
    // valid = (v > 0) && (q <= max representable code): the ordered
    // compares are false for NaN, the second one also rejects +inf, float
    // overflow of the multiply, and off-grid codes (see quantize_code).
    // Invalid lanes still go through the arithmetic below (defined,
    // garbage) and are masked to 0 at the end.
    const __m256 valid = _mm256_and_ps(
      _mm256_cmp_ps(v, vzero, _CMP_GT_OQ), _mm256_cmp_ps(q, vqmax, _CMP_LE_OQ));
    // Round half up by adding 0.5 and truncating (cvtt), matching
    // quantize_code bit-for-bit and independent of the FPU rounding mode.
    const __m256i c = _mm256_add_epi32(_mm256_cvttps_epi32(_mm256_add_ps(q, vhalf)), vone);
    _mm256_storeu_si256(
      reinterpret_cast<__m256i *>(code + i),
      _mm256_and_si256(c, _mm256_castps_si256(valid)));
  }
  for (; i < n; ++i) {
    code[i] = quantize_code(data[i], inv_step);
  }
}

// Lane-exact vector form of med_predict_u32: unsigned min/max plus the two
// compares blended in the same priority order as the branches (c >= max
// first), with the gradient a+b-c wrapping mod 2^32 like the scalar code.
__attribute__((target("avx2"))) inline __m256i med_predict_u32_avx2(
  __m256i a, __m256i b, __m256i c)
{
  const __m256i mx = _mm256_max_epu32(a, b);
  const __m256i mn = _mm256_min_epu32(a, b);
  const __m256i c_ge_mx = _mm256_cmpeq_epi32(_mm256_max_epu32(c, mx), c);
  const __m256i c_le_mn = _mm256_cmpeq_epi32(_mm256_min_epu32(c, mn), c);
  const __m256i grad = _mm256_sub_epi32(_mm256_add_epi32(a, b), c);
  return _mm256_blendv_epi8(_mm256_blendv_epi8(grad, mx, c_le_mn), mn, c_ge_mx);
}

__attribute__((target("avx2"))) void qpred_residuals_avx2(
  const uint32_t * code, uint32_t w, uint32_t h, uint32_t * zres)
{
  zres[0] = zigzag32(code[0]);
  for (uint32_t x = 1; x < w; ++x) {
    zres[x] = zigzag32(code[x] - code[x - 1]);
  }
  for (uint32_t y = 1; y < h; ++y) {
    const size_t row = static_cast<size_t>(y) * w;
    zres[row] = zigzag32(code[row] - code[row - w]);
    uint32_t x = 1;
    for (; x + 8 <= w; x += 8) {
      const size_t i = row + x;
      const __m256i left = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(code + i - 1));
      const __m256i up = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(code + i - w));
      const __m256i upl = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(code + i - w - 1));
      const __m256i cur = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(code + i));
      const __m256i s = _mm256_sub_epi32(cur, med_predict_u32_avx2(left, up, upl));
      const __m256i z = _mm256_xor_si256(_mm256_slli_epi32(s, 1), _mm256_srai_epi32(s, 31));
      _mm256_storeu_si256(reinterpret_cast<__m256i *>(zres + i), z);
    }
    for (; x < w; ++x) {
      const size_t i = row + x;
      zres[i] = zigzag32(code[i] - med_predict_u32(code[i - 1], code[i - w], code[i - w - 1]));
    }
  }
}
#endif  // DEPTH_CODEC_AVX2_DISPATCH

inline uint8_t * put_varint(uint8_t * p, uint32_t z)
{
  do {
    uint8_t byte = z & 0x7F;
    z >>= 7;
    if (z) {byte |= 0x80;}
    *p++ = byte;
  } while (z);
  return p;
}

inline float code_to_float(uint32_t c, double step_d)
{
  return c == 0 ?
         std::numeric_limits<float>::quiet_NaN() :
         static_cast<float>(static_cast<double>(c - 1) * step_d);
}

#if defined(DEPTH_CODEC_AVX2_DISPATCH)
__attribute__((target("avx2"))) void qpred_codes_to_floats_avx2(
  const uint32_t * code, size_t n, float step, float * out)
{
  const double step_d = static_cast<double>(step);
  const __m256d vstep_d = _mm256_set1_pd(step_d);
  const __m256i vzero = _mm256_setzero_si256();
  const __m256i vone = _mm256_set1_epi32(1);
  const __m256 vnan = _mm256_set1_ps(std::numeric_limits<float>::quiet_NaN());
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const __m256i c = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(code + i));
    // Codes >= 2^31 cannot come from this encoder (it rejects them as
    // invalid) but a hostile/corrupt stream can produce them; the epi32
    // conversions would misread them as negative, so such blocks take the
    // scalar path.
    if (_mm256_movemask_ps(_mm256_castsi256_ps(c)) != 0) {
      for (int k = 0; k < 8; ++k) {
        out[i + k] = code_to_float(code[i + k], step_d);
      }
      continue;
    }
    // Multiply in DOUBLE, then narrow to float, exactly like
    // code_to_float: int32->double is exact and the two roundings (double
    // product, float narrowing) match the scalar path bit-for-bit, so a
    // blob decodes identically with or without AVX2. c == 0 lanes run
    // through the arithmetic (c-1 -> -1 -> -step, garbage) and are blended
    // to NaN at the end.
    const __m256i cm1 = _mm256_sub_epi32(c, vone);
    const __m256d lo =
      _mm256_mul_pd(_mm256_cvtepi32_pd(_mm256_castsi256_si128(cm1)), vstep_d);
    const __m256d hi =
      _mm256_mul_pd(_mm256_cvtepi32_pd(_mm256_extracti128_si256(cm1, 1)), vstep_d);
    const __m256 f = _mm256_insertf128_ps(
      _mm256_castps128_ps256(_mm256_cvtpd_ps(lo)), _mm256_cvtpd_ps(hi), 1);
    const __m256i is_invalid = _mm256_cmpeq_epi32(c, vzero);
    _mm256_storeu_ps(out + i, _mm256_blendv_ps(f, vnan, _mm256_castsi256_ps(is_invalid)));
  }
  for (; i < n; ++i) {
    out[i] = code_to_float(code[i], step_d);
  }
}
#endif  // DEPTH_CODEC_AVX2_DISPATCH

void qpred_codes_to_floats(const uint32_t * code, size_t n, float step, float * out)
{
#if defined(DEPTH_CODEC_AVX2_DISPATCH)
  if (cpu_has_avx2()) {
    qpred_codes_to_floats_avx2(code, n, step, out);
    return;
  }
#endif
  const double step_d = static_cast<double>(step);
  for (size_t i = 0; i < n; ++i) {
    out[i] = code_to_float(code[i], step_d);
  }
}

void qpred_encode(
  const float * data, uint32_t w, uint32_t h, float step, int level,
  EncodeScratch & scratch, std::vector<uint8_t> & out)
{
  const size_t n = static_cast<size_t>(w) * h;
  std::vector<uint32_t> & code = scratch.words32;
  std::vector<uint32_t> & zres = scratch.residuals;
  code.resize(n);
  zres.resize(n);
  const float inv_step = 1.0f / step;

  static_assert(sizeof(float) == 4, "IEEE-754 single precision expected");
  uint32_t step_bits;
  std::memcpy(&step_bits, &step, 4);
  put_u32(out, step_bits);

  if (n > 0) {
#if defined(DEPTH_CODEC_AVX2_DISPATCH)
    if (cpu_has_avx2()) {
      qpred_quantize_avx2(data, n, inv_step, code.data());
      qpred_residuals_avx2(code.data(), w, h, zres.data());
    } else  // NOLINT(readability/braces) -- scalar block shared with non-x86
#endif
    {
      qpred_quantize_scalar(data, n, inv_step, code.data());
      qpred_residuals_scalar(code.data(), w, h, zres.data());
    }
  }

  // Varint emission through a raw pointer into a worst-case-sized reused
  // buffer (no per-byte push_back). Groups of 8 whose residuals all fit one
  // byte -- the overwhelmingly common case on smooth depth -- skip the
  // per-value continuation logic entirely.
  std::vector<uint8_t> & plain = scratch.plain;
  plain.resize(n * kMaxVarintBytes);
  uint8_t * p = plain.data();
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const uint32_t any = zres[i] | zres[i + 1] | zres[i + 2] | zres[i + 3] |
      zres[i + 4] | zres[i + 5] | zres[i + 6] | zres[i + 7];
    if (any < 0x80) {
      for (int k = 0; k < 8; ++k) {
        p[k] = static_cast<uint8_t>(zres[i + k]);
      }
      p += 8;
    } else {
      for (int k = 0; k < 8; ++k) {
        p = put_varint(p, zres[i + k]);
      }
    }
  }
  for (; i < n; ++i) {
    p = put_varint(p, zres[i]);
  }
  zstd_append(out, plain.data(), static_cast<size_t>(p - plain.data()), level);
}

// `step` comes from the uncompressed payload prefix, already validated by
// parse_blob (positive, finite); `comp` points past it, at the zstd frame.
void qpred_decode(
  const uint8_t * comp, size_t comp_size, float * out, uint32_t w, uint32_t h, float step)
{
  const size_t n = static_cast<size_t>(w) * h;
  DecodeScratch & scratch = tl_decode_scratch();
  std::vector<uint8_t> & plain = scratch.plain;
  // Upper bound: n varints of at most 5 bytes each.
  zstd_unpack(comp, comp_size, n * kMaxVarintBytes, /*exact_size=*/false, plain);

  // Pass 1: parse the varint stream into a flat residual plane. Splitting
  // this from the reconstruction lets pass 2 run a wavefront over rows
  // (variable-length varints cannot be indexed randomly, a flat plane can)
  // and keeps the single-byte fast path branch-predictable.
  std::vector<uint32_t> & zres = scratch.residuals;
  zres.resize(n);
  {
    const uint8_t * p = plain.data();
    const uint8_t * const end = plain.data() + plain.size();
    for (size_t i = 0; i < n; ++i) {
      if (p == end) {
        fail("qpred_decode: truncated varint stream");
      }
      uint8_t byte = *p++;
      if (byte < 0x80) {  // ~all residuals on smooth depth
        zres[i] = byte;
        continue;
      }
      uint64_t z = byte & 0x7F;
      unsigned shift = 7;
      while (true) {
        if (p == end) {
          fail("qpred_decode: truncated varint stream");
        }
        byte = *p++;
        z |= static_cast<uint64_t>(byte & 0x7F) << shift;
        if (!(byte & 0x80)) {
          break;
        }
        shift += 7;
        if (shift >= kMaxVarintBytes * 7) {
          fail("qpred_decode: varint overflow");
        }
      }
      if (z > 0xFFFFFFFFull) {
        fail("qpred_decode: varint overflow");
      }
      zres[i] = static_cast<uint32_t>(z);
    }
    if (p != end) {
      fail("qpred_decode: trailing bytes after varint stream");
    }
  }
  if (n == 0) {
    return;
  }

  // Pass 2: reconstruct the code plane. Serial along a row (each prediction
  // needs the code just reconstructed), so process R rows along a skewed
  // diagonal -- the same wavefront as predict_unpack, with the u32
  // branch-form MED matching the encoder.
  std::vector<uint32_t> & code = scratch.words32;
  code.resize(n);
  const auto unstep = [&](size_t i, uint32_t pred) -> uint32_t {
      const uint32_t c = pred + unzigzag32(zres[i]);
      code[i] = c;
      return c;
    };
  unstep(0, 0);
  for (uint32_t x = 1; x < w; ++x) {
    unstep(x, code[x - 1]);
  }
  constexpr uint32_t R = 4;
  uint32_t y = 1;
  if (w >= 2 * R) {
    for (; y + R <= h; y += R) {
      uint32_t left[R] = {};
      for (uint32_t t = 0; t < R; ++t) {  // ramp-up diagonals
        for (uint32_t r = 0; r <= t; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint32_t * up = code.data() + row - w;
          left[r] = (c == 0) ?
            unstep(row, up[0]) :
            unstep(row + c, med_predict_u32(left[r], up[c], up[c - 1]));
        }
      }
      for (uint32_t t = R; t < w; ++t) {  // steady state
        for (uint32_t r = 0; r < R; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint32_t * up = code.data() + row - w;
          left[r] = unstep(row + c, med_predict_u32(left[r], up[c], up[c - 1]));
        }
      }
      for (uint32_t t = w; t < w + R - 1; ++t) {  // drain
        for (uint32_t r = t - w + 1; r < R; ++r) {
          const uint32_t c = t - r;
          const size_t row = static_cast<size_t>(y + r) * w;
          const uint32_t * up = code.data() + row - w;
          left[r] = unstep(row + c, med_predict_u32(left[r], up[c], up[c - 1]));
        }
      }
    }
  }
  for (; y < h; ++y) {  // strip remainder, or narrow images
    const size_t row = static_cast<size_t>(y) * w;
    uint32_t left = unstep(row, code[row - w]);
    const uint32_t * up = code.data() + row - w;
    for (uint32_t x = 1; x < w; ++x) {
      left = unstep(row + x, med_predict_u32(left, up[x], up[x - 1]));
    }
  }

  // Pass 3: codes -> floats (code 0 -> NaN per REP 118).
  qpred_codes_to_floats(code.data(), n, step, out);
}

// ---- public self-describing blob -------------------------------------------
// Layout: 'D' 'P' 'C' '1' | u8 name_len | name | i32 level | u32 w | u32 h
//         | payload. Identical to the standalone depth_codec library.
constexpr char kMethod32[] = "dpred";
constexpr char kMethod16[] = "dpred16";
constexpr char kMethodFpred[] = "fpred";
constexpr char kMethodQpred[] = "qpred";

// Reset `out` (keeping its capacity) and write the blob header; the payload
// is then appended in place by the caller.
void begin_blob(
  std::vector<uint8_t> & out, const char * name, int level, uint32_t width, uint32_t height)
{
  out.clear();
  const char magic[4] = {'D', 'P', 'C', '1'};
  out.insert(out.end(), magic, magic + 4);
  const size_t name_len = std::strlen(name);
  out.push_back(static_cast<uint8_t>(name_len));
  out.insert(out.end(), name, name + name_len);
  put_u32(out, static_cast<uint32_t>(level));
  put_u32(out, width);
  put_u32(out, height);
}

enum class Method
{
  kDpred,   // 32FC1, lossless, dictionary + MED on indices
  kDpred16,  // 16UC1, lossless, MED directly on values
  kFpred,   // 32FC1, lossless, dictionary-free MED on float total order
  kQpred    // 32FC1, lossy, configurable-step quantization + varint residuals
};

struct ParsedBlob
{
  BlobHeader header;
  Method method;
  const uint8_t * payload;
  size_t payload_size;
};

ParsedBlob parse_blob(const uint8_t * blob, size_t size)
{
  if (size < 5 || std::memcmp(blob, "DPC1", 4) != 0) {
    fail("depth_codec: bad magic");
  }
  size_t pos = 4;
  const uint8_t name_len = blob[pos++];
  if (pos + name_len + 12 > size) {
    fail("depth_codec: truncated header");
  }
  const std::string method(reinterpret_cast<const char *>(blob + pos), name_len);
  pos += name_len;
  ParsedBlob parsed;
  parsed.header.quantization_step = 0.0f;
  if (method == kMethod32) {
    parsed.method = Method::kDpred;
    parsed.header.format = PixelFormat::FLOAT32;
  } else if (method == kMethod16) {
    parsed.method = Method::kDpred16;
    parsed.header.format = PixelFormat::UINT16;
  } else if (method == kMethodFpred) {
    parsed.method = Method::kFpred;
    parsed.header.format = PixelFormat::FLOAT32;
  } else if (method == kMethodQpred) {
    parsed.method = Method::kQpred;
    parsed.header.format = PixelFormat::FLOAT32;
  } else {
    fail("depth_codec: unknown method (blob from a newer library?)");
  }
  pos += 4;  // level: not needed to decode
  parsed.header.width = get_u32(blob + pos);
  parsed.header.height = get_u32(blob + pos + 4);
  pos += 8;
  parsed.payload = blob + pos;
  parsed.payload_size = size - pos;
  if (parsed.method == Method::kQpred) {
    // The quantization step is an uncompressed payload prefix, so the
    // header alone (this function; no decompression) fully describes how
    // to decode and interpret the blob. Validate it here: a header
    // consumer must never see a nonsensical step.
    if (parsed.payload_size < 4) {
      fail("depth_codec: truncated qpred payload");
    }
    float step;
    const uint32_t step_bits = get_u32(parsed.payload);
    std::memcpy(&step, &step_bits, 4);
    if (!std::isfinite(step) || step <= 0.0f) {
      fail("depth_codec: invalid qpred quantization step");
    }
    parsed.header.quantization_step = step;
    parsed.payload += 4;
    parsed.payload_size -= 4;
  }
  return parsed;
}

}  // namespace

void encode_depth(
  const float * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, int zstd_level)
{
  const int level = std::clamp(zstd_level, 1, 3);
  const size_t n = static_cast<size_t>(width) * height;
  const uint32_t * words = reinterpret_cast<const uint32_t *>(data);
  EncodeScratch & scratch = tl_encode_scratch();
  // The dictionary build decides the blob method, so it runs before the
  // header is written: dpred when the distinct-value count fits 16 bits,
  // fpred (dictionary-free, cardinality-unlimited) when it overflows --
  // which is the common case for full-precision float stereo depth.
  if (build_value_dict(words, n, scratch.dict_table, scratch.entries, scratch.idx)) {
    begin_blob(out, kMethod32, level, width, height);
    dpred_encode_with_dict(width, height, level, scratch, out);
  } else {
    begin_blob(out, kMethodFpred, level, width, height);
    fpred_encode(data, width, height, level, scratch, out);
  }
}

void encode_depth_quantized(
  const float * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, float step, int zstd_level)
{
  if (!std::isfinite(step) || step <= 0.0f) {
    fail("encode_depth_quantized: step must be a positive finite value");
  }
  const int level = std::clamp(zstd_level, 1, 3);
  EncodeScratch & scratch = tl_encode_scratch();
  begin_blob(out, kMethodQpred, level, width, height);
  qpred_encode(data, width, height, step, level, scratch, out);
}

// 16UC1 payload: a zstd frame of [low plane | high plane] (2 * w * h bytes).
// The pixel value is already a small monotone integer -- its own sorted
// dictionary index -- so no dictionary (and no overflow fallback) is needed.
void encode_depth16(
  const uint16_t * data, uint32_t width, uint32_t height,
  std::vector<uint8_t> & out, int zstd_level)
{
  const int level = std::clamp(zstd_level, 1, 3);
  const size_t n = static_cast<size_t>(width) * height;
  std::vector<uint8_t> & plain = tl_encode_scratch().plain;
  plain.resize(n * 2);
  predict_pack(data, width, height, plain.data(), plain.data() + n);
  begin_blob(out, kMethod16, level, width, height);
  zstd_append(out, plain.data(), plain.size(), level);
}

BlobHeader read_header(const uint8_t * blob, size_t size)
{
  return parse_blob(blob, size).header;
}

void decode_depth(const uint8_t * blob, size_t size, float * out)
{
  const ParsedBlob parsed = parse_blob(blob, size);
  if (parsed.header.format != PixelFormat::FLOAT32) {
    fail("decode_depth: blob is not 32FC1");
  }
  switch (parsed.method) {
    case Method::kFpred:
      fpred_decode(
        parsed.payload, parsed.payload_size, out, parsed.header.width, parsed.header.height);
      break;
    case Method::kQpred:
      qpred_decode(
        parsed.payload, parsed.payload_size, out, parsed.header.width, parsed.header.height,
        parsed.header.quantization_step);
      break;
    default:
      dpred_decode(
        parsed.payload, parsed.payload_size, out, parsed.header.width, parsed.header.height);
      break;
  }
}

void decode_depth16(const uint8_t * blob, size_t size, uint16_t * out)
{
  const ParsedBlob parsed = parse_blob(blob, size);
  if (parsed.header.format != PixelFormat::UINT16) {
    fail("decode_depth16: blob is not 16UC1");
  }
  const size_t n = static_cast<size_t>(parsed.header.width) * parsed.header.height;
  // The 16UC1 payload has no variable-size header: the decompressed size is
  // known exactly upfront, so zstd_unpack enforces it exactly and no
  // after-the-fact size check is needed here.
  std::vector<uint8_t> & plain = tl_decode_scratch().plain;
  zstd_unpack(parsed.payload, parsed.payload_size, n * 2, /*exact_size=*/true, plain);
  predict_unpack(
    plain.data(), plain.data() + n, parsed.header.width, parsed.header.height, out);
}

}  // namespace depth_codec
