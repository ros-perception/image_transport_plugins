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


#include "avif_image_transport/avif_subscriber.hpp"

#include <avif/avif.h>

#include <chrono>
#include <sstream>
#include <iostream>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>


namespace avif_image_transport
{
AVIFSubscriber::AVIFSubscriber()
: logger_(rclcpp::get_logger("AVIFSubscriber"))
{
  this->decoder_ = avifDecoderCreate();

  RCLCPP_DEBUG(logger_, "AVIFSubscriber");
}

std::string AVIFSubscriber::getTransportName() const
{
  return "avif";
}

void AVIFSubscriber::subscribeImpl(
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

void AVIFSubscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  avifDecoderSetIOMemory(
    this->decoder_, &msg->data[0],
    msg->data.size());
  avifImage * avif_image = avifImageCreateEmpty();
  avifDecoderReadMemory(
    this->decoder_, avif_image, &msg->data[0],
    msg->data.size());

  auto result = std::make_shared<sensor_msgs::msg::Image>();
  result->height = avif_image->height;
  result->width = avif_image->width;
  result->is_bigendian = false;
  result->step = avif_image->yuvRowBytes[0] * 3;
  result->encoding = "rgb8";
  result->data.resize(result->height * result->width * 3);

  avifRGBImage rgba;
  avifRGBImageSetDefaults(&rgba, avif_image);
  rgba.format = AVIF_RGB_FORMAT_BGR;
  rgba.rowBytes = result->step;
  rgba.depth = 8;
  rgba.pixels = reinterpret_cast<uint8_t *>(const_cast<uint8_t *>(&result->data[0]));
  avifImageYUVToRGB(avif_image, &rgba);

  user_cb(result);
  avifImageDestroy(avif_image);
}
}  // namespace avif_image_transport
