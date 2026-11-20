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


#ifndef ZSTD_IMAGE_TRANSPORT__ZSTD_PUBLISHER_HPP_
#define ZSTD_IMAGE_TRANSPORT__ZSTD_PUBLISHER_HPP_

#include <string>
#include <vector>
#include <unordered_set>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/node_interfaces.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

#include "zstd_image_transport/zstd_common.hpp"

namespace zstd_image_transport
{

using CompressedImage = sensor_msgs::msg::CompressedImage;

/// Publisher plugin of the zstd transport.
/**
 * Compresses the data of the images, without loss, and publishes them in the
 * \<base_topic\>/zstd topic. The compression level is a parameter of the node, see the
 * documentation of the package.
 */
class ZstdPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  ZstdPublisher();
  ~ZstdPublisher() override = default;

protected:
  /// Advertise the topic and declare the parameters of the transport.
  void advertiseImpl(
    image_transport::RequiredInterfaces node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    rclcpp::PublisherOptions options) final;

  /// Compress an image with the current parameters and publish it.
  void publish(
    const sensor_msgs::msg::Image & message,
    const PublisherT & publisher) const override;

  /// Logger of the plugin
  rclcpp::Logger logger_;
  /// Parameters interface of the node that publishes the images
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param_interface_;
  /// Base interface of the node that publishes the images
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;

private:
  std::vector<std::string> parameters_;
  std::unordered_set<std::string> deprecated_parameters_;

  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
    pre_set_parameter_callback_handle_;

  void declareParameter(
    const std::string & base_name,
    const ParameterDefinition & definition);

  void preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace zstd_image_transport

#endif  // ZSTD_IMAGE_TRANSPORT__ZSTD_PUBLISHER_HPP_
