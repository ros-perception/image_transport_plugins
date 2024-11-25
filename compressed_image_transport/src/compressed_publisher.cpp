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

#include "compressed_image_transport/compressed_publisher.hpp"

#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "compressed_image_transport/compression_common.hpp"

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

enum compressedParameters
{
  FORMAT = 0,
  PNG_LEVEL,
  JPEG_QUALITY,
  TIFF_RESOLUTION_UNIT,
  TIFF_XDPI,
  TIFF_YDPI
};

const struct ParameterDefinition kParameters[] =
{
  {  // FORMAT - Compression format to use "jpeg", "png" or "tiff".
    ParameterValue("jpeg"),
    ParameterDescriptor()
    .set__name("format")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("Compression method")
    .set__read_only(false)
    .set__additional_constraints("Supported values: [jpeg, png, tiff]")
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
  {  // JPEG_QUALITY - JPEG Quality from 0 to 100 (higher is better quality).
    ParameterValue(static_cast<int>(95)),  // Default to OpenCV default of 95.
    ParameterDescriptor()
    .set__name("jpeg_quality")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Image quality for JPEG format")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(1)
        .set__to_value(100)
        .set__step(1)})
  },
  {  // TIFF_RESOLUTION_UNIT - TIFF resolution unit, can be one of "none", "inch", "centimeter".
    ParameterValue("inch"),
    ParameterDescriptor()
    .set__name("tiff.res_unit")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("tiff resolution unit")
    .set__read_only(false)
    .set__additional_constraints("Supported values: [none, inch, centimeter]")
  },
  {  // TIFF_XDPI
    ParameterValue(static_cast<int>(-1)),
    ParameterDescriptor()
    .set__name("tiff.xdpi")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("tiff xdpi")
    .set__read_only(false)
  },
  {  // TIFF_YDPI
    ParameterValue(static_cast<int>(-1)),
    ParameterDescriptor()
    .set__name("tiff.ydpi")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("tiff ydpi")
    .set__read_only(false)
  }
};

void CompressedPublisher::advertiseImpl(
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
  auto callback = std::bind(&CompressedPublisher::onParameterEvent, this, std::placeholders::_1,
                            node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<callbackT>(node,
      callback);

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void CompressedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Fresh Configuration
  std::string cfg_format = node_->get_parameter(parameters_[FORMAT]).get_value<std::string>();
  int cfg_png_level = node_->get_parameter(parameters_[PNG_LEVEL]).get_value<int64_t>();
  int cfg_jpeg_quality = node_->get_parameter(parameters_[JPEG_QUALITY]).get_value<int64_t>();
  std::string cfg_tiff_res_unit =
    node_->get_parameter(parameters_[TIFF_RESOLUTION_UNIT]).get_value<std::string>();
  int cfg_tiff_xdpi = node_->get_parameter(parameters_[TIFF_XDPI]).get_value<int64_t>();
  int cfg_tiff_ydpi = node_->get_parameter(parameters_[TIFF_YDPI]).get_value<int64_t>();

  // Compressed image message
  sensor_msgs::msg::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Compression settings
  std::vector<int> params;

  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (cfg_format == "jpeg") {
    encodingFormat = JPEG;
  } else if (cfg_format == "png") {
    encodingFormat = PNG;
  } else if (cfg_format == "tiff") {
    encodingFormat = TIFF;
  }

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  switch (encodingFormat) {
    // JPEG Compression
    case JPEG:
      {
        params.reserve(2);
        params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
        params.emplace_back(cfg_jpeg_quality);

        // Update ros message format header
        compressed.format += "; jpeg compressed ";

        // Check input format
        if ((bitDepth == 8) || (bitDepth == 16)) {
          // Target image format
          std::string targetFormat;
          if (enc::isColor(message.encoding)) {
            // convert color images to BGR8 format
            targetFormat = "bgr8";
            compressed.format += targetFormat;
          } else {
            // convert gray images to mono8 format
            targetFormat = "mono8";
            compressed.format += targetFormat;
          }

          // OpenCV-ros bridge
          try {
            std::shared_ptr<CompressedPublisher> tracked_object;
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object,
              targetFormat);

            // Compress image
            if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params)) {
              float cRatio = static_cast<float>(cv_ptr->image.rows * cv_ptr->image.cols *
                cv_ptr->image.elemSize() / compressed.data.size());
              RCLCPP_DEBUG(logger_,
                "Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)",
                cRatio, compressed.data.size());
            } else {
              RCLCPP_ERROR(logger_, "cv::imencode (jpeg) failed on input image");
            }
          } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(logger_, "%s", e.what());
          } catch (cv::Exception & e) {
            RCLCPP_ERROR(logger_, "%s", e.what());
          }

          // Publish message
          publish_fn(compressed);
        } else {
          RCLCPP_ERROR(logger_,
            "Compressed Image Transport - JPEG compression requires 8/16-bit color format "
            "(input format is: %s)", message.encoding.c_str());
        }
        break;
      }
    // PNG Compression
    case PNG:
      {
        params.reserve(2);
        params.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
        params.emplace_back(cfg_png_level);

        // Update ros message format header
        compressed.format += "; png compressed ";

        // Check input format
        if ((bitDepth == 8) || (bitDepth == 16)) {
          // Target image format
          std::stringstream targetFormat;
          if (enc::isColor(message.encoding)) {
          // convert color images to RGB domain
            targetFormat << "bgr";
            if (enc::hasAlpha(message.encoding)) {
              targetFormat << "a";
            }
            targetFormat << bitDepth;
            compressed.format += targetFormat.str();
          }

          // OpenCV-ros bridge
          try {
            std::shared_ptr<CompressedPublisher> tracked_object;
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object,
              targetFormat.str());

            // Compress image
            if (cv::imencode(".png", cv_ptr->image, compressed.data, params)) {
              float cRatio = static_cast<float>(cv_ptr->image.rows * cv_ptr->image.cols *
                cv_ptr->image.elemSize() / compressed.data.size());
              RCUTILS_LOG_DEBUG(
                "Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)",
                cRatio, compressed.data.size());
            } else {
              RCUTILS_LOG_ERROR("cv::imencode (png) failed on input image");
            }
          } catch (cv_bridge::Exception & e) {
            RCUTILS_LOG_ERROR("%s", e.what());
            return;
          } catch (cv::Exception & e) {
            RCUTILS_LOG_ERROR("%s", e.what());
            return;
          }

          // Publish message
          publish_fn(compressed);
        } else {
          RCUTILS_LOG_ERROR(
          "Compressed Image Transport - PNG compression requires 8/16-bit "
          "encoded color format (input format is: %s)",
            message.encoding.c_str());
        }
        break;
      }
    // TIFF Compression
    case TIFF:
      {
        // Update ros message format header
        compressed.format += "; tiff compressed ";
        int res_unit = -1;
        // See https://gitlab.com/libtiff/libtiff/-/blob/v4.3.0/libtiff/tiff.h#L282-284
        if (cfg_tiff_res_unit == "inch") {
          res_unit = 2;
        } else if (cfg_tiff_res_unit == "centimeter") {
          res_unit = 3;
        } else if (cfg_tiff_res_unit == "none") {
          res_unit = 1;
        } else {
          RCLCPP_WARN(
          logger_,
          "tiff.res_unit parameter should be either 'inch', 'centimeter' or 'none'; "
          "defaulting to 'inch'. Found '%s'", cfg_tiff_res_unit.c_str());
        }
        params.reserve(3);
        params.emplace_back(cv::IMWRITE_TIFF_XDPI);
        params.emplace_back(cfg_tiff_xdpi);
        params.emplace_back(cv::IMWRITE_TIFF_YDPI);
        params.emplace_back(cfg_tiff_ydpi);
        params.emplace_back(cv::IMWRITE_TIFF_RESUNIT);
        params.emplace_back(res_unit);

        // Check input format
        if ((bitDepth == 8) || (bitDepth == 16) || (bitDepth == 32)) {
          // OpenCV-ros bridge
          try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, nullptr, "");

            // Compress image
            if (cv::imencode(".tiff", cv_ptr->image, compressed.data, params)) {
              float cRatio = static_cast<float>(cv_ptr->image.rows * cv_ptr->image.cols *
                cv_ptr->image.elemSize() / compressed.data.size());
              RCUTILS_LOG_DEBUG(
                "Compressed Image Transport - Codec: tiff, Compression Ratio: 1:%.2f (%lu bytes)",
                cRatio, compressed.data.size());
            } else {
              RCUTILS_LOG_ERROR("cv::imencode (tiff) failed on input image");
            }
          } catch (cv_bridge::Exception & e) {
            RCUTILS_LOG_ERROR("%s", e.what());
            return;
          } catch (cv::Exception & e) {
            RCUTILS_LOG_ERROR("%s", e.what());
            return;
          }

          // Publish message
          publish_fn(compressed);
        } else {
          RCUTILS_LOG_ERROR(
          "Compressed Image Transport - TIFF compression requires 8/16/32-bit encoded color format "
          "(input format is: %s)", message.encoding.c_str());
        }
        break;
      }

    default:
      RCUTILS_LOG_ERROR("Unknown compression type '%s', valid options are 'jpeg', 'png' and 'tiff'",
        cfg_format.c_str());
      break;
  }
}

void CompressedPublisher::declareParameter(
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

void CompressedPublisher::onParameterEvent(
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
