// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include "zlib_image_transport/zlib_publisher.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "zlib_wrapper.hpp"

namespace zlib_image_transport
{

enum zlibParameters
{
  ZLIB_LEVEL = 0,
};

const struct ParameterDefinition kParameters[] =
{
  {
    // ZLIB_LEVEL - zlib Compression Level from 0 to 9. Higher value = smaller size.
    ParameterValue(static_cast<int>(6)),
    ParameterDescriptor()
    .set__name("zlib_level")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Compression level for zlib format (0=none, 1=fastest, 9=best compression)")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(9)
        .set__step(1)})
  },
};

ZlibPublisher::ZlibPublisher()
: logger_(rclcpp::get_logger("ZlibPublisher"))
{
}

std::string ZlibPublisher::getTransportName() const
{
  return "zlib";
}

void ZlibPublisher::advertiseImpl(
  image_transport::RequiredInterfaces node_interfaces,
  const std::string & base_topic,
  rclcpp::QoS custom_qos,
  rclcpp::PublisherOptions options)
{
  node_param_interface_ = node_interfaces.get_node_parameters_interface();
  node_base_interface_ = node_interfaces.get_node_base_interface();
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node_interfaces, base_topic, custom_qos, options);

  unsigned int ns_len =
    std::string(node_interfaces.get_node_base_interface()->get_namespace()).length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  if (ns_len > 1) {
    pre_set_parameter_callback_handle_ =
      node_param_interface_->add_pre_set_parameters_callback(std::bind(
        &ZlibPublisher::preSetParametersCallback,
        this, std::placeholders::_1));
  }

  for (const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void ZlibPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  const int cfg_level =
    node_param_interface_->get_parameter(parameters_[ZLIB_LEVEL]).as_int();

  // Pre-allocate output buffer to the zlib worst-case bound.
  const std::size_t bound = zlib_wrapper::compressBound(message.data.size());
  const std::size_t metadata =
    4 +   // height
    4 +   // width
    1 +   // is_bigendian
    4 +   // step
    4 +   // encoding string length
    message.encoding.size();

  sensor_msgs::msg::CompressedImage compressed;
  compressed.data.resize(metadata + bound);

  const std::size_t compressed_size = zlib_wrapper::compress(
    &compressed.data[metadata], bound,
    message.data.data(), message.data.size(),
    cfg_level);

  if (compressed_size == 0) {
    RCLCPP_ERROR(logger_, "zlib compression failed");
    return;
  }

  compressed.data.resize(metadata + compressed_size);

  // ---- Metadata header (little-endian) ----
  compressed.data[0] = static_cast<uint8_t>(message.height & 0xFF);
  compressed.data[1] = static_cast<uint8_t>((message.height >> 8) & 0xFF);
  compressed.data[2] = static_cast<uint8_t>((message.height >> 16) & 0xFF);
  compressed.data[3] = static_cast<uint8_t>((message.height >> 24) & 0xFF);

  compressed.data[4] = static_cast<uint8_t>(message.width & 0xFF);
  compressed.data[5] = static_cast<uint8_t>((message.width >> 8) & 0xFF);
  compressed.data[6] = static_cast<uint8_t>((message.width >> 16) & 0xFF);
  compressed.data[7] = static_cast<uint8_t>((message.width >> 24) & 0xFF);

  compressed.data[8] = message.is_bigendian;

  compressed.data[9]  = static_cast<uint8_t>(message.step & 0xFF);
  compressed.data[10] = static_cast<uint8_t>((message.step >> 8) & 0xFF);
  compressed.data[11] = static_cast<uint8_t>((message.step >> 16) & 0xFF);
  compressed.data[12] = static_cast<uint8_t>((message.step >> 24) & 0xFF);

  compressed.data[13] = static_cast<uint8_t>(message.encoding.size() & 0xFF);
  compressed.data[14] = static_cast<uint8_t>((message.encoding.size() >> 8) & 0xFF);
  compressed.data[15] = static_cast<uint8_t>((message.encoding.size() >> 16) & 0xFF);
  compressed.data[16] = static_cast<uint8_t>((message.encoding.size() >> 24) & 0xFF);

  memcpy(&compressed.data[17], message.encoding.data(), message.encoding.size());
  // -----------------------------------------

  compressed.header = message.header;
  compressed.format = "zlib";
  publisher->publish(compressed);
}

void ZlibPublisher::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_param_interface_->declare_parameter(
      param_name, definition.defaultValue,
      definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_param_interface_->get_parameter(param_name).get_parameter_value();
  }

  if (std::string(node_base_interface_->get_namespace()).length() > 1) {
    const std::string deprecated_dot_name = "." + base_name + "." + transport_name + "." +
      definition.descriptor.name;
    deprecated_parameters_.insert(deprecated_dot_name);

    try {
      node_param_interface_->declare_parameter(
        deprecated_dot_name, param_value, definition.descriptor);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    }
  }
}

void ZlibPublisher::preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rclcpp::Parameter> new_parameters;

  for (auto & param : parameters) {
    const auto & param_name = param.get_name();

    if (deprecated_parameters_.find(param_name) != deprecated_parameters_.end()) {
      auto non_dot_prefixed_name = param_name.substr(1);
      RCLCPP_WARN_STREAM(
        logger_,
        "parameter `" << param_name <<
          "` with leading dot character is deprecated; use: `" <<
          non_dot_prefixed_name << "` instead");
      new_parameters.push_back(
        rclcpp::Parameter(non_dot_prefixed_name, param.get_parameter_value()));
    }

    if (std::find(parameters_.begin(), parameters_.end(), param_name) != parameters_.end()) {
      new_parameters.emplace_back("." + param_name, param.get_parameter_value());
    }
  }

  parameters.insert(parameters.end(), new_parameters.begin(), new_parameters.end());
}

}  // namespace zlib_image_transport
