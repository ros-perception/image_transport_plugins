// Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef THEORA_IMAGE_TRANSPORT__THEORA_PUBLISHER_HPP_
#define THEORA_IMAGE_TRANSPORT__THEORA_PUBLISHER_HPP_

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <image_transport/node_interfaces.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <std_msgs/msg/header.hpp>
#include <theora_image_transport/msg/packet.hpp>

#include "theora_image_transport/compression_common.hpp"

namespace theora_image_transport
{

/// Publisher plugin of the theora transport.
/**
 * Encodes the images as a Theora video stream and publishes its packets in the
 * \<base_topic\>/theora topic. The encoder is configured with parameters of the node, see the
 * documentation of the package.
 */
class TheoraPublisher
  : public image_transport::SimplePublisherPlugin<theora_image_transport::msg::Packet>
{
public:
  TheoraPublisher();
  ~TheoraPublisher() override;

protected:
  /// Advertise the topic and declare the parameters of the transport.
  void advertiseImpl(
    image_transport::RequiredInterfaces node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    rclcpp::PublisherOptions options) override;

  // TODO(anyone): Callback to send header packets to new clients

  /// Encode an image and publish the packets of the stream.
  void publish(
    const sensor_msgs::msg::Image & message,
    const PublisherT & publisher) const override;

  /// Apply the current values of the parameters to the encoder, if they have changed.
  void refreshConfig() const;
  /// Whether the parameters have changed since the last call to refreshConfig()
  mutable bool refreshConfigNeeded;

  /// Create the encoder, and publish the stream headers, if needed for this image.
  bool ensureEncodingContext(
    const sensor_msgs::msg::Image & image,
    const PublisherT & publisher) const;
  /// Convert a packet of the Theora stream into a message.
  void oggPacketToMsg(
    const std_msgs::msg::Header & header,
    const ogg_packet & oggpacket,
    theora_image_transport::msg::Packet & msg) const;
  /// Apply the key frame frequency to the encoder.
  void updateKeyframeFrequency() const;

  // Some data is preserved across calls to publish(), but from the user's perspective publish() is
  // "logically const"
  /// Configuration of the encoder
  mutable th_info encoder_setup_;
  /// Maximum distance between key frames
  mutable ogg_uint32_t keyframe_frequency_;
  /// The encoder, created when the first image is published
  mutable std::shared_ptr<th_enc_ctx> encoding_context_;
  /// The header packets of the current stream
  mutable std::vector<theora_image_transport::msg::Packet> stream_header_;

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
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameter_callback_handle_;

  void declareParameter(
    const std::string & base_name,
    const ParameterDefinition & definition);

  void preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters);

  void postSetParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace theora_image_transport

#endif  // THEORA_IMAGE_TRANSPORT__THEORA_PUBLISHER_HPP_
