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

#include "zstd_image_transport/zstd_publisher.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

#include <span>  // NOLINT(build/include_order) cpplint misclassifies <span>

#include <rclcpp/rclcpp.hpp>

#include "byte_order.hpp"
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

void ZstdPublisher::advertiseImpl(
  image_transport::RequiredInterfaces node_interfaces,
  const std::string & base_topic,
  rclcpp::QoS custom_qos,
  rclcpp::PublisherOptions options)
{
  node_param_interface_ = node_interfaces.get_node_parameters_interface();
  node_base_interface_ = node_interfaces.get_node_base_interface();
  using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
  Base::advertiseImpl(node_interfaces, base_topic, custom_qos, options);

  unsigned int ns_len =
    std::string(node_interfaces.get_node_base_interface()->get_namespace()).length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  if (ns_len > 1) {
    // Add pre set parameter callback to handle deprecated parameters
    pre_set_parameter_callback_handle_ =
      node_param_interface_->add_pre_set_parameters_callback(
        [this](std::vector<rclcpp::Parameter> & parameters) {
          preSetParametersCallback(parameters);
        });
  }

  for (const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void ZstdPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  // Fresh Configuration
  int cfg_zstd_level =
    node_param_interface_->get_parameter(
      parameters_[ZSTD_LEVEL]).as_int();

  zlib::Comp comp(static_cast<zlib::Comp::Level>(cfg_zstd_level), true);
  const std::vector<uint8_t> payload = comp.Process(
    std::span<const uint8_t>(message.data.data(), message.data.size()), true);

  auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();

  // Little-endian header: height(4) width(4) is_bigendian(1) step(4)
  //                       encoding_size(4) encoding(encoding_size)
  const std::size_t metadata = 4 + 4 + 1 + 4 + 4 + message.encoding.size();
  compressed->data.resize(metadata + payload.size());

  const std::span<uint8_t> header(compressed->data.data(), metadata);
  store_le(header.subspan<0, 4>(), message.height);
  store_le(header.subspan<4, 4>(), message.width);
  header[8] = static_cast<uint8_t>(message.is_bigendian);
  store_le(header.subspan<9, 4>(), message.step);
  store_le(header.subspan<13, 4>(), static_cast<uint32_t>(message.encoding.size()));
  std::copy(message.encoding.begin(), message.encoding.end(), compressed->data.begin() + 17);

  std::copy(payload.begin(), payload.end(), compressed->data.begin() + metadata);

  // Compressed image message
  compressed->header = message.header;
  compressed->format = "zstd";
  publisher->publish(std::move(compressed));
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
    param_value = node_param_interface_->declare_parameter(
      param_name, definition.defaultValue,
      definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_param_interface_->get_parameter(param_name).get_parameter_value();
  }

  // TODO(anyone): Remove deprecated parameters after Lyrical release
  if (std::string(node_base_interface_->get_namespace()).length() > 1) {
    // deprecated parameters starting with the dot character (e.g. .image_raw.compressed.format)
    const std::string deprecated_dot_name = "." + base_name + "." + transport_name + "." +
      definition.descriptor.name;
    deprecated_parameters_.insert(deprecated_dot_name);

    try {
      node_param_interface_->declare_parameter(deprecated_dot_name, param_value,
          definition.descriptor);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    }
  }
}

void ZstdPublisher::preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rclcpp::Parameter> new_parameters;

  for (auto & param : parameters) {
    const auto & param_name = param.get_name();

    // Check if this is a deprecated dot-prefixed parameter for our transport
    if (deprecated_parameters_.contains(param_name)) {
      auto non_dot_prefixed_name = param_name.substr(1);
      RCLCPP_WARN_STREAM(logger_,
            "parameter `" << param_name << "` with leading dot character is deprecated; use: `" <<
            non_dot_prefixed_name << "` instead");
      new_parameters.push_back(
          rclcpp::Parameter(non_dot_prefixed_name, param.get_parameter_value()));
    }

    // Check if this is a normal parameter for our transport
    if (std::find(parameters_.begin(), parameters_.end(), param_name) != parameters_.end()) {
      // Also update the dot-prefixed parameter
      new_parameters.emplace_back("." + param_name, param.get_parameter_value());
    }
  }

  parameters.insert(parameters.end(), new_parameters.begin(), new_parameters.end());
}
}  // namespace zstd_image_transport
