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

// Codec-level round-trip tests: they exercise the OpenCV image codecs the
// compressed transport relies on (JPEG / PNG), with no ROS node or transport.

#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace
{

// Build a bgr8 image with a deterministic per-pixel pattern.
cv::Mat makeBgr8(int w, int h)
{
  cv::Mat img(h, w, CV_8UC3);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      img.at<cv::Vec3b>(y, x) = cv::Vec3b(
        static_cast<uchar>((x * 7) & 0xFF),
        static_cast<uchar>((y * 5) & 0xFF),
        static_cast<uchar>((x + y) & 0xFF));
    }
  }
  return img;
}

int maxAbsDiff(const cv::Mat & a, const cv::Mat & b)
{
  cv::Mat diff;
  cv::absdiff(a, b, diff);
  double maxval = 0.0;
  cv::minMaxLoc(diff.reshape(1), nullptr, &maxval);
  return static_cast<int>(maxval);
}

}  // namespace

// PNG is lossless: encode -> decode must reproduce the image exactly.
TEST(CompressedCodecRoundTrip, PngIsLossless)
{
  const cv::Mat original = makeBgr8(32, 24);
  std::vector<uchar> buffer;
  const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
  ASSERT_TRUE(cv::imencode(".png", original, buffer, params));

  const cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.size(), original.size());
  EXPECT_EQ(decoded.type(), original.type());
  EXPECT_EQ(maxAbsDiff(original, decoded), 0);
}

// JPEG is lossy: structure exact, pixel values within a tolerance.
TEST(CompressedCodecRoundTrip, JpegApproximate)
{
  const cv::Mat original = makeBgr8(32, 24);
  std::vector<uchar> buffer;
  const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
  ASSERT_TRUE(cv::imencode(".jpg", original, buffer, params));

  const cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_COLOR);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.size(), original.size());
  EXPECT_LE(maxAbsDiff(original, decoded), 20) << "JPEG round-trip drifted more than expected";
}

// A single-channel 16-bit image (depth-like) must survive PNG losslessly.
TEST(CompressedCodecRoundTrip, Png16BitIsLossless)
{
  cv::Mat original(24, 32, CV_16UC1);
  for (int y = 0; y < original.rows; ++y) {
    for (int x = 0; x < original.cols; ++x) {
      original.at<uint16_t>(y, x) = static_cast<uint16_t>((x * 337 + y * 71) & 0xFFFF);
    }
  }
  std::vector<uchar> buffer;
  ASSERT_TRUE(cv::imencode(".png", original, buffer));

  const cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.type(), original.type());
  EXPECT_EQ(maxAbsDiff(original, decoded), 0);
}
