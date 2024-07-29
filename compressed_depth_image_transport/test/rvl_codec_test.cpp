// Copyright (c) 2023
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

#include "compressed_depth_image_transport/rvl_codec.hpp"
#include <gtest/gtest.h>

TEST(RvlCodecTest, reciprocalTestEmpty) {
  const int size = 1000000;
  std::vector<uint16_t> original(size);
  std::vector<unsigned char> compressed(3 * size + 4);
  std::vector<uint16_t> decompressed(size);
  compressed_depth_image_transport::RvlCodec rvl;

  // Constant depth.
  const int validDepth = 42;
  std::fill(original.begin(), original.end(), validDepth);
  rvl.CompressRVL(&original[0], &compressed[0], size);
  rvl.DecompressRVL(&compressed[0], &decompressed[0], size);
  EXPECT_TRUE(std::equal(original.begin(), original.end(), decompressed.begin()));

  // Totally invalid depth.
  const int invalidDepth = 0;
  std::fill(original.begin(), original.end(), invalidDepth);
  rvl.CompressRVL(&original[0], &compressed[0], size);
  rvl.DecompressRVL(&compressed[0], &decompressed[0], size);
  EXPECT_TRUE(std::equal(original.begin(), original.end(), decompressed.begin()));

  // Empty depth.
  EXPECT_EQ(rvl.CompressRVL(NULL, NULL, 0), 0);
  rvl.DecompressRVL(NULL, NULL, 0);  // should not die.
}

TEST(RvlCodecTest, reciprocalTestRandom) {
  const int size = 1000000;
  std::vector<uint16_t> original(size);
  std::vector<unsigned char> compressed(3 * size + 4);
  std::vector<uint16_t> decompressed(size);
  compressed_depth_image_transport::RvlCodec rvl;

  // Populate depths with random size of runs with random values.
  for (int i = 0; i < size; ) {
    int length = std::min<int>(rand() % 10, size - i);  // NOLINT
    int value = rand() % 10;  // NOLINT
    std::fill(&original[i], &original[i] + length, value);
    i += length;
  }

  const int compressedSize =
    rvl.CompressRVL(&original[0], &compressed[0], size);
  EXPECT_GT(compressedSize, 0);
  EXPECT_LT(compressedSize, compressed.size());
  rvl.DecompressRVL(&compressed[0], &decompressed[0], size);
  EXPECT_TRUE(std::equal(original.begin(), original.end(), decompressed.begin()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
