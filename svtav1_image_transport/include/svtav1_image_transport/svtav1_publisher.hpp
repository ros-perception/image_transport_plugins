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

#ifndef SVTAV1_IMAGE_TRANSPORT__SVTAV1_PUBLISHER_HPP_
#define SVTAV1_IMAGE_TRANSPORT__SVTAV1_PUBLISHER_HPP_

#include <svt-av1/EbSvtAv1Enc.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

#include "svtav1_image_transport/svtav1_common.hpp"

#include <opencv2/core/mat.hpp>

namespace svtav1_image_transport
{
using CompressedImage = sensor_msgs::msg::CompressedImage;
using ParameterEvent = rcl_interfaces::msg::ParameterEvent;

class SVTAV1Publisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  SVTAV1Publisher();
  ~SVTAV1Publisher() override;
  std::string getTransportName() const override;

protected:
  // Overridden to set up reconfigure server
  void advertiseImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options) override;

  void publish(
    const sensor_msgs::msg::Image & message,
    const PublishFn & publish_fn) const override;

  void Initialize(int width, int height) const;

  rclcpp::Logger logger_;
  rclcpp::Node * node_;

private:
  void set_default_svt_configuration(int width, int height) const;

  std::vector<std::string> parameters_;

  void declareParameter(
    const std::string & base_name,
    const ParameterDefinition & definition);

  /* SVT-AV1 Encoder Handle */
  mutable EbComponentType * svt_encoder{nullptr};
  /* SVT-AV1 configuration */
  mutable EbSvtAv1EncConfiguration * svt_config{nullptr};
  mutable uint64_t frame_count;
  // int dts_offset;
  mutable EbBufferHeaderType * input_buf{nullptr};

  mutable std::mutex mutex;
  mutable cv::Mat ycrcb_planes[3];
};

}  // namespace svtav1_image_transport

#endif  // SVTAV1_IMAGE_TRANSPORT__SVTAV1_PUBLISHER_HPP_
