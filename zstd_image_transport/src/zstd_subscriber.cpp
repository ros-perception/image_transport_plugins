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


#include "zstd_image_transport/zstd_subscriber.hpp"

#include <cstring>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>

#include "zstd_wrapper.hpp"

namespace zstd_image_transport
{
ZstdSubscriber::ZstdSubscriber()
: logger_(rclcpp::get_logger("ZstdSubscriber"))
{
}

void ZstdSubscriber::subscribeImpl(
  image_transport::RequiredInterfaces node_interfaces,
  const std::string & base_topic,
  const Callback & callback,
  rclcpp::QoS custom_qos,
  rclcpp::SubscriptionOptions options)
{
  logger_ = node_interfaces.get_node_logging_interface()->get_logger();
  typedef image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::subscribeImpl(node_interfaces, base_topic, callback, custom_qos, options);
}

void ZstdSubscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  constexpr std::size_t kFixedHeaderSize = 4 + 4 + 1 + 4 + 4;  // = 17 bytes

  if (msg->data.size() < kFixedHeaderSize) {
    RCLCPP_ERROR(logger_, "Compressed image too small to contain header");
    return;
  }

  auto result = std::make_shared<sensor_msgs::msg::Image>();

  // ---- Decode fixed metadata header (little-endian) ----
  result->height =
    (static_cast<uint32_t>(msg->data[3]) << 24) |
    (static_cast<uint32_t>(msg->data[2]) << 16) |
    (static_cast<uint32_t>(msg->data[1]) << 8) |
    static_cast<uint32_t>(msg->data[0]);

  result->width =
    (static_cast<uint32_t>(msg->data[7]) << 24) |
    (static_cast<uint32_t>(msg->data[6]) << 16) |
    (static_cast<uint32_t>(msg->data[5]) << 8) |
    static_cast<uint32_t>(msg->data[4]);

  result->is_bigendian = msg->data[8];

  result->step =
    (static_cast<uint32_t>(msg->data[12]) << 24) |
    (static_cast<uint32_t>(msg->data[11]) << 16) |
    (static_cast<uint32_t>(msg->data[10]) << 8) |
    static_cast<uint32_t>(msg->data[9]);

  const uint32_t encoding_size =
    (static_cast<uint32_t>(msg->data[16]) << 24) |
    (static_cast<uint32_t>(msg->data[15]) << 16) |
    (static_cast<uint32_t>(msg->data[14]) << 8) |
    static_cast<uint32_t>(msg->data[13]);
  // ------------------------------------------------------

  const std::size_t metadata = kFixedHeaderSize + encoding_size;
  if (msg->data.size() < metadata) {
    RCLCPP_ERROR(logger_, "Compressed image data truncated (encoding string missing)");
    return;
  }

  result->encoding.resize(encoding_size);
  memcpy(&result->encoding[0], &msg->data[17], encoding_size);

  // Pointer and size of the actual compressed payload.
  const uint8_t * compressed_ptr = &msg->data[metadata];
  const std::size_t compressed_size = msg->data.size() - metadata;

  if (compressed_size == 0) {
    RCLCPP_ERROR(logger_, "Compressed image payload is empty");
    return;
  }

  // Lazily initialise the reusable decompression context.
  if (!decompressor_) {
    decompressor_ = std::make_unique<zstd_wrapper::Decompressor>();
  }

  // zstd always writes the content size in the frame header for single-block
  // compression, so this gives us the exact destination buffer size.
  const std::size_t decompressed_size =
    zstd_wrapper::Decompressor::getDecompressedSize(compressed_ptr, compressed_size);

  if (decompressed_size == 0) {
    RCLCPP_ERROR(logger_, "Could not read decompressed size from zstd frame header");
    return;
  }

  result->data.resize(decompressed_size);
  const std::size_t actual_size = decompressor_->decompress(
    result->data.data(), decompressed_size,
    compressed_ptr, compressed_size);

  if (actual_size == 0) {
    RCLCPP_ERROR(logger_, "zstd decompression failed");
    return;
  }

  result->data.resize(actual_size);
  result->header = msg->header;

  user_cb(result);
}
}  // namespace zstd_image_transport
