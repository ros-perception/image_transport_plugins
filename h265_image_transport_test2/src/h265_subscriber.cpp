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


#include "h265_image_transport/h265_subscriber.hpp"

#include <avif/avif.h>

#include <memory>
#include <mutex>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <libde265/de265.h>
#include <libde265/image.h>

#include <cv_bridge/cv_bridge.hpp>


namespace h265_image_transport
{
H265Subscriber::H265Subscriber()
: logger_(rclcpp::get_logger("H265Subscriber"))
{
  RCLCPP_DEBUG(logger_, "H265Subscriber");
  de265_init();

  this->ctx_ = de265_new_decoder();
  de265_set_parameter_int(this->ctx_, DE265_DECODER_PARAM_ACCELERATION_CODE, de265_acceleration_SCALAR);
  de265_set_verbosity(3);

}

std::string H265Subscriber::getTransportName() const
{
  return "h265";
}

void H265Subscriber::subscribeImpl(
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

void H265Subscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  std::unique_lock<std::mutex> guard(this->mutex);

  std::cout << "internalCallback " << msg->data.size() << std::endl;

  de265_flush_data(this->ctx_);

  static int frame = 0;

  auto err = de265_push_NAL(this->ctx_, &msg->data[0], msg->data.size(), frame++, (void *)2);
  if (err != DE265_OK) {
    std::cout << "Error" << std::endl;
  }

  int more = 1;
  while (more) {
    more = 0;

    // decode some more

    auto err = de265_decode(this->ctx_, &more);

    if (err != DE265_OK) {
      std::cout << "err " << err << std::endl;
    }

    const de265_image * img_decode = de265_get_next_picture(this->ctx_);
    if (img_decode) {

      int width = de265_get_image_width(img_decode, 0);
      int height = de265_get_image_height(img_decode, 0);
      std::cout << "width " << width << " height: " << height << std::endl;

      cv::Mat restored_image(height, width, CV_8UC3);

      // int stride;
      // const uint8_t* data;
      // data = de265_get_image_plane(img_decode, 0, &stride);

      // for (int y = 0; y < height; y++) {
      //   memcpy(&restored_image.data[y], data + y*stride, width);
      // }

      std::vector<cv::Mat> ch;
      ch.resize(3);

      int stride;

      for (int c = 0; c < 3; c++) {
        int h265channel;
        switch (c) {
          case 0: h265channel = 2; break; // R
          case 1: h265channel = 0; break; // G
          case 2: h265channel = 1; break; // B
        }

        const uint8_t * p = img_decode->get_image_plane(h265channel);
        stride = img_decode->get_image_stride(h265channel);

        cv::extractChannel(restored_image, ch[c], c); // extract specific channel

        for (int y = 0; y < restored_image.rows; y++) {
          memcpy(&ch[c].data[y * restored_image.rows], p, restored_image.cols);
          p += stride;
        }
      }
      std_msgs::msg::Header header; // empty header
      header.stamp = this->node_->now();

      auto result = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, restored_image);
      auto img_msg = result.toImageMsg(); // from cv_bridge to sensor_msgs::Image
      user_cb(img_msg);
    } else {
      std::cout << "nullptr " << std::endl;

    }
  }
}
}  // namespace h265_image_transport
