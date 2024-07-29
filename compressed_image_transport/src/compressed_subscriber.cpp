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

#include <limits>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  node_ = node;
  logger_ = node->get_logger();
  typedef image_transport::SimpleSubscriberPlugin<CompressedImage> Base;
  Base::subscribeImpl(node, base_topic, callback, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  using paramCallbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto paramCallback = std::bind(&CompressedSubscriber::onParameterEvent, this,
      std::placeholders::_1, node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<paramCallbackT>(node,
      paramCallback);

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
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), cfg_imdecode_flag);

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
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
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
  std::string mode = node_->get_parameter(parameters_[MODE]).get_value<std::string>();

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

  // deprecated non-scoped parameter name (e.g. image_raw.format)
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

void CompressedSubscriber::onParameterEvent(
  ParameterEvent::SharedPtr event, std::string full_name,
  std::string base_name)
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
    // e.g. `color.image_raw.` + `compressed` + `format`
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

}  // namespace compressed_image_transport
