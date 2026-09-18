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

#ifndef DEPTHZ_IMAGE_TRANSPORT__DEPTHZ_PUBLISHER_HPP_
#define DEPTHZ_IMAGE_TRANSPORT__DEPTHZ_PUBLISHER_HPP_

#include <string>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/node_interfaces.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

namespace depthz_image_transport
{

using CompressedImage = sensor_msgs::msg::CompressedImage;

class DepthzPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  DepthzPublisher();
  ~DepthzPublisher() override = default;

  std::string getTransportName() const override
  {
    return "depthz";
  }

protected:
  void advertiseImpl(
    image_transport::RequiredInterfaces node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    rclcpp::PublisherOptions options) final;

  void publish(
    const sensor_msgs::msg::Image & message,
    const PublisherT & publisher) const override;

  rclcpp::Logger logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param_interface_;

private:
  std::string level_param_name_;
  std::string quantization_param_name_;
};

}  // namespace depthz_image_transport

#endif  // DEPTHZ_IMAGE_TRANSPORT__DEPTHZ_PUBLISHER_HPP_
