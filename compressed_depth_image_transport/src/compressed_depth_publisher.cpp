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

#include "compressed_depth_image_transport/compressed_depth_publisher.hpp"

#include <cstddef>
#include <string>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "compressed_depth_image_transport/compression_common.hpp"
#include "compressed_depth_image_transport/codec.hpp"


namespace compressed_depth_image_transport
{

enum compressedDepthParameters
{
  FORMAT = 0,
  DEPTH_MAX,
  DEPTH_QUANTIZATION,
  PNG_LEVEL
};

const struct ParameterDefinition kParameters[] =
{
  {  // FORMAT - Compression format to use "png" or "rvl".
    ParameterValue("png"),
    ParameterDescriptor()
    .set__name("format")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("Compression method")
    .set__read_only(false)
    .set__additional_constraints("Supported values: [png, rvl]")
  },
  {  // DEPTH_MAX - Maximum depth value (meter)
    ParameterValue(static_cast<double>(10.0)),
    ParameterDescriptor()
    .set__name("depth_max")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
    .set__description("Maximum depth value (meter)")
    .set__read_only(false)
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(1.0)
        .set__to_value(100.0)
        .set__step(0.0)})
  },
  {  // DEPTH_QUANTIZATION - Depth value at which the sensor accuracy is 1 m (Kinect: >75)
    ParameterValue(static_cast<double>(100.0)),
    ParameterDescriptor()
    .set__name("depth_quantization")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
    .set__description("Depth value at which the sensor accuracy is 1 m (Kinect: >75)")
    .set__read_only(false)
    .set__floating_point_range(
      {rcl_interfaces::msg::FloatingPointRange()
        .set__from_value(1.0)
        .set__to_value(150.0)
        .set__step(0.0)})
  },
  {  // PNG_LEVEL - PNG Compression Level from 0 to 9.  A higher value means a smaller size.
    ParameterValue(static_cast<int>(3)),  // Default to OpenCV default of 3
    ParameterDescriptor()
    .set__name("png_level")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Compression level for PNG format")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(9)
        .set__step(1)})
  },
};

void CompressedDepthPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rclcpp::QoS custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;

  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  uint ns_prefix_len = ns_len > 1 ? ns_len + 1 : ns_len;
  std::string param_base_name = base_topic.substr(ns_prefix_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  if (ns_len > 1) {
    // Add pre set parameter callback to handle deprecated parameters
    pre_set_parameter_callback_handle_ =
      node->add_pre_set_parameters_callback(std::bind(
      &CompressedDepthPublisher::preSetParametersCallback, this, std::placeholders::_1));
  }

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void CompressedDepthPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  // Fresh Configuration
  std::string cfg_format = node_->get_parameter(parameters_[FORMAT]).get_value<std::string>();
  double cfg_depth_max = node_->get_parameter(parameters_[DEPTH_MAX]).get_value<double>();
  double cfg_depth_quantization =
    node_->get_parameter(parameters_[DEPTH_QUANTIZATION]).get_value<double>();
  int cfg_png_level = node_->get_parameter(parameters_[PNG_LEVEL]).get_value<int64_t>();

  sensor_msgs::msg::CompressedImage::SharedPtr compressed_image =
    encodeCompressedDepthImage(message,
                               cfg_format,
                               cfg_depth_max,
                               cfg_depth_quantization,
                               cfg_png_level);
  if (compressed_image) {
    publisher->publish(*compressed_image);
  }
}

void CompressedDepthPublisher::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressedDepth.png_level)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(param_name, definition.defaultValue,
        definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }

  // TODO(anyone): Remove deprecated parameters after Lyrical release
  if (node_->get_effective_namespace().length() > 1) {
    // deprecated parameters starting with the dot character (e.g. .image_raw.compressed.format)
    const std::string deprecated_dot_name = "." + base_name + "." + transport_name + "." +
      definition.descriptor.name;
    deprecated_parameters_.insert(deprecated_dot_name);

    try {
      node_->declare_parameter(deprecated_dot_name, param_value, definition.descriptor);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    }
  }
}

void CompressedDepthPublisher::preSetParametersCallback(
  std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rclcpp::Parameter> new_parameters;

  for (auto & param : parameters) {
    const auto & param_name = param.get_name();

    // Check if this is a deprecated dot-prefixed parameter for our transport
    if (deprecated_parameters_.find(param_name) != deprecated_parameters_.end()) {
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
}  // namespace compressed_depth_image_transport
