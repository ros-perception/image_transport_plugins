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

#include "zstd_image_transport/zstd_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "zlib_cpp.hpp"

namespace zstd_image_transport
{

enum zstdParameters
{
  ZSTD_LEVEL = 0,
};

const struct ParameterDefinition kParameters[] =
{
  {
    // ZSTD_LEVEL - ZSTD Compression Level from 0 to 9. A higher value means a smaller size.
    ParameterValue(static_cast<int>(3)),
    ParameterDescriptor()
    .set__name("zstd_level")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Compression level for ZSTD format")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(9)
        .set__step(1)})
  },
};

ZstdPublisher::ZstdPublisher()
: logger_(rclcpp::get_logger("ZstdPublisher"))
{
}

std::string ZstdPublisher::getTransportName() const
{
  return "zstd";
}

void ZstdPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  unsigned int ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  for (const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void ZstdPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Fresh Configuration
  int cfg_zstd_level = node_->get_parameter(parameters_[ZSTD_LEVEL]).get_value<int64_t>();

  zlib::Comp comp(static_cast<zlib::Comp::Level>(cfg_zstd_level), true);
  auto g_compressed_data =
    comp.Process(&message.data[0], message.data.size(), true);

  size_t total_size = 0;
  for (const auto & data : g_compressed_data) {
    total_size += data->size;
  }

  sensor_msgs::msg::CompressedImage compressed;

  int metadata = 4 + 4 + 1 + 4 + 4 + message.encoding.size();

  compressed.data.resize(total_size + metadata);

  size_t index = metadata;
  for (const auto & data : g_compressed_data) {
    memcpy(&compressed.data[index], data->ptr, data->size);
    index += data->size;
  }

  compressed.data[0] = static_cast<uint8_t>(message.height & 0xFF);
  compressed.data[1] = static_cast<uint8_t>(message.height >> 8) & 0xFF;
  compressed.data[2] = static_cast<uint8_t>(message.height >> 16) & 0xFF;
  compressed.data[3] = static_cast<uint8_t>(message.height >> 24) & 0xFF;

  compressed.data[4] = static_cast<uint8_t>(message.width & 0xFF);
  compressed.data[5] = static_cast<uint8_t>(message.width >> 8) & 0xFF;
  compressed.data[6] = static_cast<uint8_t>(message.width >> 16) & 0xFF;
  compressed.data[7] = static_cast<uint8_t>(message.width >> 24) & 0xFF;

  compressed.data[8] = message.is_bigendian;

  compressed.data[9] = static_cast<uint8_t>(message.step & 0xFF);
  compressed.data[10] = static_cast<uint8_t>(message.step >> 8) & 0xFF;
  compressed.data[11] = static_cast<uint8_t>(message.step >> 16) & 0xFF;
  compressed.data[12] = static_cast<uint8_t>(message.step >> 24) & 0xFF;

  compressed.data[13] = static_cast<uint8_t>(message.encoding.size() & 0xFF);
  compressed.data[14] = static_cast<uint8_t>(message.encoding.size() >> 8) & 0xFF;
  compressed.data[15] = static_cast<uint8_t>(message.encoding.size() >> 16) & 0xFF;
  compressed.data[16] = static_cast<uint8_t>(message.encoding.size() >> 24) & 0xFF;

  memcpy(&compressed.data[17], &message.encoding[0], message.encoding.size());

  // Compressed image message
  compressed.header = message.header;
  compressed.format = "zstd";
  publish_fn(compressed);
}

void ZstdPublisher::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.zstd.zstd_level)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(
      param_name, definition.defaultValue,
      definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }
}
}  // namespace zstd_image_transport
