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

#include "depthz_image_transport/depthz_publisher.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "depth_codec.hpp"

namespace depthz_image_transport
{

DepthzPublisher::DepthzPublisher()
: logger_(rclcpp::get_logger("DepthzPublisher"))
{
}

void DepthzPublisher::advertiseImpl(
  image_transport::RequiredInterfaces node_interfaces,
  const std::string & base_topic,
  rclcpp::QoS custom_qos,
  rclcpp::PublisherOptions options)
{
  node_param_interface_ = node_interfaces.get_node_parameters_interface();
  typedef image_transport::SimplePublisherPlugin<CompressedImage> Base;
  Base::advertiseImpl(node_interfaces, base_topic, custom_qos, options);

  // Transport-scoped parameter (e.g. image_raw.depthz.zstd_level).
  const unsigned int ns_len =
    std::string(node_interfaces.get_node_base_interface()->get_namespace()).length();
  std::string param_base_name = base_topic.substr(ns_len);
  // A non-root namespace leaves a leading '/' after the substr (root eats
  // it, ns_len == 1), which would yield dot-prefixed parameter names.
  if (!param_base_name.empty() && param_base_name.front() == '/') {
    param_base_name.erase(0, 1);
  }
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  level_param_name_ = param_base_name + "." + getTransportName() + ".zstd_level";
  quantization_param_name_ = param_base_name + "." + getTransportName() + ".quantization";

  const auto declare = [this](
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor) {
      try {
        node_param_interface_->declare_parameter(name, default_value, descriptor);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        RCLCPP_DEBUG(logger_, "%s was previously declared", name.c_str());
      }
    };

  rcl_interfaces::msg::ParameterDescriptor level_descriptor;
  level_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  level_descriptor.description = "zstd level of the depthz entropy stage (1-3)";
  level_descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
    .set__from_value(1)
    .set__to_value(3)
    .set__step(1)};
  declare(level_param_name_, rclcpp::ParameterValue(1), level_descriptor);

  rcl_interfaces::msg::ParameterDescriptor quantization_descriptor;
  quantization_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  quantization_descriptor.description =
    "32FC1 quantization step in millimeters; the decoded depth is within "
    "+/- half this step of the input. 0.0 = bit-exact lossless. Ignored for "
    "16UC1 input (already integer, compressed losslessly).";
  quantization_descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
    .set__from_value(0.0)
    .set__to_value(100.0)
    .set__step(0.0)};
  declare(quantization_param_name_, rclcpp::ParameterValue(0.1), quantization_descriptor);
}

void DepthzPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  const bool is_32f = message.encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  const bool is_16u = message.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  if (!is_32f && !is_16u) {
    RCLCPP_ERROR_ONCE(
      logger_, "depthz transport supports only 32FC1 and 16UC1 depth images, got '%s'. "
      "Use compressed or zstd for other encodings.", message.encoding.c_str());
    return;
  }
  if (message.is_bigendian) {
    RCLCPP_ERROR_ONCE(logger_, "depthz transport does not support big-endian images");
    return;
  }

  const int level = static_cast<int>(
    node_param_interface_->get_parameter(level_param_name_).as_int());
  const double quantization_mm =
    node_param_interface_->get_parameter(quantization_param_name_).as_double();

  const uint32_t w = message.width;
  const uint32_t h = message.height;
  const size_t bpp = depth_codec::bytes_per_pixel(
    is_32f ? depth_codec::PixelFormat::FLOAT32 : depth_codec::PixelFormat::UINT16);
  // Validate step/data size for every message, not just the padded-row
  // case below: even when step == w*bpp exactly, a truncated data buffer
  // (e.g. a malformed or hand-built message) must not flow into the
  // encoder, which trusts width/height/step to size its reads.
  if (message.step < w * bpp || message.data.size() < static_cast<size_t>(message.step) * h) {
    RCLCPP_ERROR_ONCE(logger_, "inconsistent image step/size, dropping frame");
    return;
  }

  const uint8_t * src = message.data.data();
  // Reused across frames (publish is const, so thread_local rather than a
  // member), matching the codec's own scratch-buffer pattern.
  static thread_local std::vector<uint8_t> packed;
  if (message.step != w * bpp) {  // rows are padded: repack contiguously
    packed.resize(static_cast<size_t>(w) * h * bpp);
    for (uint32_t y = 0; y < h; ++y) {
      std::memcpy(
        packed.data() + static_cast<size_t>(y) * w * bpp,
        src + static_cast<size_t>(y) * message.step,
        static_cast<size_t>(w) * bpp);
    }
    src = packed.data();
  }

  try {
    auto compressed = std::make_unique<CompressedImage>();
    compressed->header = message.header;
    compressed->format = message.encoding + "; depthz";
    // The encoder writes the blob directly into the message field.
    if (is_32f && quantization_mm > 0.0) {
      // Default path: lossy, bounded error of +/- quantization/2. The
      // format string advertises the lossiness so bag consumers can tell
      // without decoding the blob (the blob itself is self-describing).
      char suffix[32];
      std::snprintf(suffix, sizeof(suffix), "; lossy %.3fmm", quantization_mm);
      compressed->format += suffix;
      depth_codec::encode_depth_quantized(
        reinterpret_cast<const float *>(src), w, h, compressed->data,
        static_cast<float>(quantization_mm * 1e-3), level);
    } else if (is_32f) {
      depth_codec::encode_depth(
        reinterpret_cast<const float *>(src), w, h, compressed->data, level);
    } else {
      // 16UC1 is already integer depth: quantizing it below its native
      // resolution would be a no-op, so it is always compressed losslessly.
      depth_codec::encode_depth16(
        reinterpret_cast<const uint16_t *>(src), w, h, compressed->data, level);
    }
    publisher->publish(std::move(compressed));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "depthz encoding failed: %s", e.what());
  }
}

}  // namespace depthz_image_transport
