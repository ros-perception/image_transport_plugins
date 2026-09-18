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

#include "depthz_image_transport/depthz_subscriber.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "depth_codec.hpp"

namespace depthz_image_transport
{

DepthzSubscriber::DepthzSubscriber()
: logger_(rclcpp::get_logger("DepthzSubscriber"))
{
}

void DepthzSubscriber::internalCallback(
  const CompressedImage::ConstSharedPtr & message,
  const Callback & user_cb)
{
  try {
    // The blob is self-describing: dimensions and pixel format come from
    // its header, and the decoder writes directly into image->data (no
    // intermediate buffer, no copy).
    const depth_codec::BlobHeader header =
      depth_codec::read_header(message->data.data(), message->data.size());
    const bool is_16u = header.format == depth_codec::PixelFormat::UINT16;
    const size_t bpp = depth_codec::bytes_per_pixel(header.format);

    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header = message->header;
    image->width = header.width;
    image->height = header.height;
    image->is_bigendian = false;
    image->encoding = is_16u ?
      sensor_msgs::image_encodings::TYPE_16UC1 : sensor_msgs::image_encodings::TYPE_32FC1;
    image->step = header.width * bpp;
    image->data.resize(static_cast<size_t>(header.width) * header.height * bpp);
    if (is_16u) {
      depth_codec::decode_depth16(
        message->data.data(), message->data.size(),
        reinterpret_cast<uint16_t *>(image->data.data()));
    } else {
      depth_codec::decode_depth(
        message->data.data(), message->data.size(),
        reinterpret_cast<float *>(image->data.data()));
    }
    user_cb(image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "depthz decoding failed: %s", e.what());
  }
}

}  // namespace depthz_image_transport
