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


#include "zstd_image_transport/zstd_subscriber.hpp"

#include <sstream>
#include <iostream>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "zlib_cpp.hpp"

namespace zstd_image_transport
{
ZstdSubscriber::ZstdSubscriber()
: logger_(rclcpp::get_logger("ZstdSubscriber"))
{
}

std::string ZstdSubscriber::getTransportName() const
{
  return "zstd";
}

void ZstdSubscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  node_ = node;
  logger_ = node->get_logger();
  typedef image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
}

void ZstdSubscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  auto result = std::make_shared<sensor_msgs::msg::Image>();

  zlib::Decomp decomp;

  int metadata = 4 + 4 + 1 + 4 + 4;

  result->height =
    (msg->data[3] << 24 ) +
    (msg->data[2] << 16 ) +
    (msg->data[1] << 8 ) +
    (msg->data[0]);

  result->width =
    (msg->data[7] << 24 ) +
    (msg->data[6] << 16 ) +
    (msg->data[5] << 8 ) +
    (msg->data[4]);

  result->is_bigendian = msg->data[8];

  result->step =
    (msg->data[12] << 24 ) +
    (msg->data[11] << 16 ) +
    (msg->data[10] << 8 ) +
    (msg->data[9]);

  uint32_t encoding_size =
    (msg->data[16] << 24 ) +
    (msg->data[15] << 16 ) +
    (msg->data[14] << 8 ) +
    (msg->data[13]);

  std::string encoding;
  result->encoding.resize(encoding_size);
  memcpy(&result->encoding[0], &msg->data[17], encoding_size);

  metadata += encoding_size;

  std::shared_ptr<zlib::DataBlock> data = zlib::AllocateData(msg->data.size());
  memcpy(data->ptr, &msg->data[metadata], msg->data.size());

  std::list<std::shared_ptr<zlib::DataBlock>> out_data_list;
  out_data_list = decomp.Process(data);

  std::shared_ptr<zlib::DataBlock> data2 = zlib::ExpandDataList(out_data_list);

  result->data.resize(data2->size);
  memcpy(&result->data[0], data2->ptr, data2->size);

  user_cb(result);
}
}  // namespace zstd_image_transport
