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

#ifndef THEORA_IMAGE_TRANSPORT__THEORA_SUBSCRIBER_HPP_
#define THEORA_IMAGE_TRANSPORT__THEORA_SUBSCRIBER_HPP_

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include <image_transport/node_interfaces.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <theora_image_transport/msg/packet.hpp>

#include "theora_image_transport/compression_common.hpp"

namespace theora_image_transport
{

/// Subscriber plugin of the theora transport.
/**
 * Subscribes to the \<base_topic\>/theora topic and decodes the Theora video stream. The stream
 * can only be decoded if it is received from its beginning.
 */
class TheoraSubscriber
  : public image_transport::SimpleSubscriberPlugin<theora_image_transport::msg::Packet>
{
public:
  TheoraSubscriber();
  virtual ~TheoraSubscriber();

protected:
  /// Subscribe to the topic and declare the parameters of the transport.
  void subscribeImpl(
    image_transport::RequiredInterfaces node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options) override;

  /// The function that does the actual decompression and calls a user supplied
  /// callback with the resulting image
  void internalCallback(
    const theora_image_transport::msg::Packet::ConstSharedPtr & msg,
    const Callback & user_cb) override;

  /// Apply the current values of the parameters to the decoder.
  void refreshConfig();

  /// Set the post-processing level of the decoder, and return the level that was applied.
  int updatePostProcessingLevel(int level);
  /// Convert a message into a packet of the Theora stream. The caller must delete ogg.packet.
  void msgToOggPacket(
    const theora_image_transport::msg::Packet & msg,
    ogg_packet & ogg);
  /// Post-processing level
  int pplevel_;
  /// Whether all the header packets of the stream have been received
  bool received_header_;
  /// Whether a key frame has been received
  bool received_keyframe_;
  /// The decoder, created when the headers have been received
  th_dec_ctx * decoding_context_;
  /// Stream information read from the headers
  th_info header_info_;
  /// Comments read from the headers
  th_comment header_comment_;
  /// Decoder setup read from the headers
  th_setup_info * setup_info_;
  /// The last decoded image, used again for duplicate frames
  sensor_msgs::msg::Image::SharedPtr latest_image_;

  /// Logger of the plugin
  rclcpp::Logger logger_;
  /// Parameters interface of the node that subscribes to the images
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param_interface_;

private:
  std::vector<std::string> parameters_;

  void declareParameter(
    const std::string & base_name,
    const ParameterDefinition & definition);
};

}  // namespace theora_image_transport

#endif  // THEORA_IMAGE_TRANSPORT__THEORA_SUBSCRIBER_HPP_
