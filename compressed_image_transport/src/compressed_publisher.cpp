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
  JPEG_COMPRESS_BAYER,
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
  {  // JPEG_COMPRESS_BAYER - allow compression of bayer encoded images.
    ParameterValue(static_cast<bool>(false)),
    ParameterDescriptor()
    .set__name("jpeg_compress_bayer")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    .set__description("Allow JPEG compression for bayer format")
    .set__read_only(false)
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
        &CompressedPublisher::preSetParametersCallback, this, std::placeholders::_1));
  }

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void CompressedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublisherT & publisher) const
{
  // Fresh Configuration
  std::string cfg_format = node_->get_parameter(parameters_[FORMAT]).get_value<std::string>();
  int cfg_png_level = node_->get_parameter(parameters_[PNG_LEVEL]).get_value<int64_t>();
  int cfg_jpeg_quality = node_->get_parameter(parameters_[JPEG_QUALITY]).get_value<int64_t>();
  bool cfg_jpeg_compress_bayer =
    node_->get_parameter(parameters_[JPEG_COMPRESS_BAYER]).get_value<bool>();
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
          } else if (enc::isBayer(message.encoding) && cfg_jpeg_compress_bayer) {
            // do not convert bayer format to mono
            targetFormat = message.encoding;
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
          publisher->publish(compressed);
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
          publisher->publish(compressed);
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
          publisher->publish(compressed);
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

void CompressedPublisher::preSetParametersCallback(std::vector<rclcpp::Parameter> & parameters)
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
}  // namespace compressed_image_transport
