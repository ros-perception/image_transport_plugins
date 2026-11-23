// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2016, Google, Inc.
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

#ifndef COMPRESSED_DEPTH_IMAGE_TRANSPORT__CODEC_HPP_
#define COMPRESSED_DEPTH_IMAGE_TRANSPORT__CODEC_HPP_

#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

// Encoding and decoding of compressed depth images.
namespace compressed_depth_image_transport
{

/// Decompress a depth image compressed by encodeCompressedDepthImage().
/**
 * The encoding of the image and the compression format are read from the
 * format field of the message.
 *
 * \param compressed_image The compressed depth image
 * \return The depth image, or a null pointer on bad input
 */
sensor_msgs::msg::Image::SharedPtr decodeCompressedDepthImage(
  const sensor_msgs::msg::CompressedImage & compressed_image);

/// Compress a depth image.
/**
 * Only single-channel 32-bit floating point images (depth in meters) and
 * single-channel 16-bit images (depth in millimeters) are supported.
 *
 * Floating point images are quantized to 16 bits before being compressed, so
 * their compression is lossy.
 *
 * \param message The depth image
 * \param format Compression format: "png" or "rvl"
 * \param depth_max Maximum depth value, in meters. Measurements further away
 *   are discarded
 * \param depth_quantization Depth value at which the sensor accuracy is 1 m.
 *   Only used for floating point images
 * \param png_level PNG compression level, from 0 to 9. Only used with the
 *   "png" format
 * \return The compressed depth image, or a null pointer on bad input
 */
sensor_msgs::msg::CompressedImage::UniquePtr encodeCompressedDepthImage(
  const sensor_msgs::msg::Image & message,
  const std::string & format = "png",
  double depth_max = 10.0,
  double depth_quantization = 100.0,
  int png_level = 9);
}  // namespace compressed_depth_image_transport

#endif  // COMPRESSED_DEPTH_IMAGE_TRANSPORT__CODEC_HPP_
