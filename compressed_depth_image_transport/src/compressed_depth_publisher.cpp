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
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;

  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  using callbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto callback = std::bind(&CompressedDepthPublisher::onParameterEvent, this,
      std::placeholders::_1, node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<callbackT>(node,
      callback);

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void CompressedDepthPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
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
    publish_fn(*compressed_image);
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

  // deprecated non-scoped parameter name (e.g. image_raw.png_level)
  const std::string deprecated_name = base_name + "." + definition.descriptor.name;
  deprecatedParameters_.push_back(deprecated_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(param_name, definition.defaultValue,
        definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }

  // transport scoped parameter as default, otherwise we would overwrite
  try {
    node_->declare_parameter(deprecated_name, param_value, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
  }
}

void CompressedDepthPublisher::onParameterEvent(
  ParameterEvent::SharedPtr event,
  std::string full_name, std::string base_name)
{
  // filter out events from other nodes
  if (event->node != full_name) {
    return;
  }

  // filter out new/changed deprecated parameters
  using EventType = rclcpp::ParameterEventsFilter::EventType;

  rclcpp::ParameterEventsFilter filter(event, deprecatedParameters_,
    {EventType::NEW, EventType::CHANGED});

  const std::string transport = getTransportName();

  // emit warnings for deprecated parameters & sync deprecated parameter value to correct
  for (auto & it : filter.get_events()) {
    const std::string name = it.second->name;

    // name was generated from base_name, has to succeed
    size_t baseNameIndex = name.find(base_name);
    size_t paramNameIndex = baseNameIndex + base_name.size();
    // e.g. `color.image_raw.` + `compressedDepth` + `png_level`
    std::string recommendedName = name.substr(0,
        paramNameIndex + 1) + transport + name.substr(paramNameIndex);

    rclcpp::Parameter recommendedValue = node_->get_parameter(recommendedName);

    // do not emit warnings if deprecated value matches
    if(it.second->value == recommendedValue.get_value_message()) {
      continue;
    }

    RCLCPP_WARN_STREAM(logger_, "parameter `" << name << "` is deprecated and ambiguous" <<
                                "; use transport qualified name `" << recommendedName << "`");

    node_->set_parameter(rclcpp::Parameter(recommendedName, it.second->value));
  }
}
}  // namespace compressed_depth_image_transport
