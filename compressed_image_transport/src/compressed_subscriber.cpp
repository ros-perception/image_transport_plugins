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

#include "compressed_image_transport/compressed_subscriber.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "compressed_image_transport/compression_common.hpp"

namespace enc = sensor_msgs::image_encodings;

using CompressedImage = sensor_msgs::msg::CompressedImage;

namespace compressed_image_transport
{

enum compressedParameters
{
  MODE = 0,
};

const struct ParameterDefinition kParameters[] =
{
  {  // MODE - OpenCV imdecode flags to use: [unchanged, gray, color]
    ParameterValue("unchanged"),
    ParameterDescriptor()
    .set__name("mode")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("OpenCV imdecode flags to use")
    .set__read_only(false)
    .set__additional_constraints("Supported values: [unchanged, gray, color]")
  }
};

void CompressedSubscriber::subscribeImpl(
  image_transport::RequiredInterfaces node_interfaces,
  const std::string & base_topic,
  const Callback & callback,
  rclcpp::QoS custom_qos,
  rclcpp::SubscriptionOptions options)
{
  node_param_interface_ = node_interfaces.get_node_parameters_interface();
  node_base_interface_ = node_interfaces.get_node_base_interface();
  logger_ = node_interfaces.get_node_logging_interface()->get_logger();
  using Base = image_transport::SimpleSubscriberPlugin<CompressedImage>;
  Base::subscribeImpl(node_interfaces, base_topic, callback, custom_qos, options);

  // Declare Parameters
  const std::string ns = node_interfaces.get_node_base_interface()->get_namespace();
  const size_t ns_len = ns.length();
  // Skip the namespace and the slash after it ("/ns/image" -> "image"). A topic outside
  // the node's namespace (absolute or remapped) only loses its leading slash.
  const std::string ns_prefix = ns_len > 1 ? ns + "/" : ns;
  std::string param_base_name = base_topic.substr(
    base_topic.starts_with(ns_prefix) ? ns_prefix.length() : 1);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  if (ns_len > 1) {
    // Add pre set parameter callback to handle deprecated parameters
    pre_set_parameter_callback_handle_ =
      node_param_interface_->add_pre_set_parameters_callback(
      std::bind(&CompressedSubscriber::preSetParametersCallback, this, std::placeholders::_1));
  }

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void CompressedSubscriber::internalCallback(
  const CompressedImage::ConstSharedPtr & message,
  const Callback & user_cb)
{
  int cfg_imdecode_flag = imdecodeFlagFromConfig();

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try {
    const auto & compressed_data = static_cast<const std::vector<unsigned char> &>(message->data);
    cv_ptr->image = cv::imdecode(compressed_data, cfg_imdecode_flag);

    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    if (split_pos == std::string::npos) {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels()) {
        case 1:
          cv_ptr->encoding = enc::MONO8;
          break;
        case 3:
          cv_ptr->encoding = enc::BGR8;
          break;
        default:
          RCLCPP_ERROR(logger_, "Unsupported number of channels: %i", cv_ptr->image.channels());
          break;
      }
    } else {
      std::string image_encoding = message->format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if (enc::isColor(image_encoding)) {
        std::string compressed_encoding = message->format.substr(split_pos);
        bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") !=
          std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGBA);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGRA);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2RGBA);
          }
        }
      }
      if (message->format.find("jpeg") != std::string::npos &&
        enc::bitDepth(image_encoding) == 16)
      {
        cv_ptr->image.convertTo(cv_ptr->image, CV_16U, 256);
      }
    }
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0)) {
    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
  }
}

int CompressedSubscriber::imdecodeFlagFromConfig()
{
  std::string mode =
    node_param_interface_->get_parameter(parameters_[MODE]).get_value<std::string>();

  if (mode == "unchanged") {
    return cv::IMREAD_UNCHANGED;
  } else if (mode == "gray") {
    return cv::IMREAD_GRAYSCALE;
  } else if (mode == "color") {
    return cv::IMREAD_COLOR;
  }

  RCLCPP_ERROR(logger_, "Unknown mode: %s, defaulting to 'unchanged", mode.c_str());

  return cv::IMREAD_UNCHANGED;
}

void CompressedSubscriber::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_param_interface_->declare_parameter(param_name, definition.defaultValue,
        definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_param_interface_->get_parameter(param_name).get_parameter_value();
  }

  // TODO(anyone): Remove deprecated parameters once a release has shipped with this warning
  if (std::string(node_base_interface_->get_namespace()).length() > 1) {
    // deprecated parameters starting with the dot character (e.g. .image_raw.compressed.mode)
    const std::string deprecated_dot_name = "." + param_name;
    deprecated_parameters_.insert(deprecated_dot_name);

    rclcpp::ParameterValue deprecated_value = param_value;
    try {
      deprecated_value = node_param_interface_->declare_parameter(deprecated_dot_name, param_value,
          definition.descriptor);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    }

    // The deprecated name was given at startup (e.g. from a parameter file): keep honoring it.
    if (deprecated_value != param_value) {
      RCLCPP_WARN_STREAM(logger_,
        "parameter `" << deprecated_dot_name << "` with leading dot character is deprecated; "
        "use: `" << param_name << "` instead");
      node_param_interface_->set_parameters({rclcpp::Parameter(param_name, deprecated_value)});
    }
  }
}

void CompressedSubscriber::preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters)
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

}  // namespace compressed_image_transport

PLUGINLIB_EXPORT_CLASS(compressed_image_transport::CompressedSubscriber,
  image_transport::SubscriberPlugin)
