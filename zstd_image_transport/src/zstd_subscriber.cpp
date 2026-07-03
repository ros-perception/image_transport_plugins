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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <span>  // NOLINT(build/include_order) cpplint misclassifies <span>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "byte_order.hpp"
#include "zlib_cpp.hpp"

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
  using Base = image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage>;
  Base::subscribeImpl(node_interfaces, base_topic, callback, custom_qos, options);
}

void ZstdSubscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  // Little-endian header: height(4) width(4) is_bigendian(1) step(4) encoding_size(4)
  constexpr std::size_t kHeaderSize = 4 + 4 + 1 + 4 + 4;
  if (msg->data.size() < kHeaderSize) {
    RCLCPP_ERROR(
      logger_, "zstd: message of %zu bytes is too small for the header", msg->data.size());
    return;
  }

  const std::span<const uint8_t> header(msg->data.data(), kHeaderSize);
  auto result = std::make_shared<sensor_msgs::msg::Image>();
  result->height = load_le<uint32_t>(header.subspan<0, 4>());
  result->width = load_le<uint32_t>(header.subspan<4, 4>());
  result->is_bigendian = header[8];
  result->step = load_le<uint32_t>(header.subspan<9, 4>());
  const uint32_t encoding_size = load_le<uint32_t>(header.subspan<13, 4>());

  const std::size_t metadata = kHeaderSize + encoding_size;
  if (msg->data.size() < metadata) {
    RCLCPP_ERROR(
      logger_, "zstd: message of %zu bytes is too small for its %u-byte encoding field",
      msg->data.size(), encoding_size);
    return;
  }
  result->encoding.assign(
    reinterpret_cast<const char *>(msg->data.data()) + kHeaderSize, encoding_size);

  zlib::Decomp decomp;
  result->data = decomp.Process(
    std::span<const uint8_t>(msg->data.data() + metadata, msg->data.size() - metadata));

  user_cb(result);
}
}  // namespace zstd_image_transport
