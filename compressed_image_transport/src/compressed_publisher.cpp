/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "compressed_image_transport/compressed_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgcodecs.hpp>

#include "compressed_image_transport/compression_common.h"

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sstream>
#include <vector>

constexpr const char* kDefaultFormat = "jpeg";
constexpr int kDefaultPngLevel = 3;
constexpr int kDefaultJpegQuality = 95;

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

void CompressedPublisher::advertiseImpl(
  rclcpp::Node* node,
  const std::string& base_topic,
  rmw_qos_profile_t custom_qos)
{
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos);

  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  std::string format_param_name = param_base_name + ".format";
  rcl_interfaces::msg::ParameterDescriptor format_description;
  format_description.name = "format";
  format_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  format_description.description = "Compression method";
  format_description.read_only = false;
  format_description.additional_constraints = "Supported values: [jpeg, png, tiff]";
  try {
    config_.format = node->declare_parameter(format_param_name, kDefaultFormat, format_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", format_param_name.c_str());
    config_.format = node->get_parameter(format_param_name).get_value<std::string>();
  }

  std::string png_level_param_name = param_base_name + ".png_level";
  rcl_interfaces::msg::ParameterDescriptor png_level_description;
  png_level_description.name = "png_level";
  png_level_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  png_level_description.description = "Compression level for PNG format";
  png_level_description.read_only = false;
  rcl_interfaces::msg::IntegerRange png_range;
  png_range.from_value = 0;
  png_range.to_value = 9;
  png_range.step = 1;
  png_level_description.integer_range.push_back(png_range);
  try {
    config_.png_level = node->declare_parameter(
      png_level_param_name, kDefaultPngLevel, png_level_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", png_level_param_name.c_str());
    config_.png_level = node->get_parameter(png_level_param_name).get_value<int64_t>();
  }

  std::string jpeg_quality_param_name = param_base_name + ".jpeg_quality";
  rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
  jpeg_quality_description.name = "jpeg_quality";
  jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  jpeg_quality_description.description = "Image quality for JPEG format";
  jpeg_quality_description.read_only = false;
  rcl_interfaces::msg::IntegerRange jpeg_range;
  jpeg_range.from_value = 1;
  jpeg_range.to_value = 100;
  jpeg_range.step = 1;
  jpeg_quality_description.integer_range.push_back(jpeg_range);
  try {
    config_.jpeg_quality = node->declare_parameter(
      jpeg_quality_param_name, kDefaultJpegQuality, jpeg_quality_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", jpeg_quality_param_name.c_str());
    config_.jpeg_quality = node->get_parameter(jpeg_quality_param_name).get_value<int64_t>();
  }

  std::string tiff_res_unit_param_name = param_base_name + ".tiff.res_unit";
  rcl_interfaces::msg::ParameterDescriptor tiff_res_unit_description;
  tiff_res_unit_description.description = "tiff resolution unit";
  tiff_res_unit_description.read_only = false;
  tiff_res_unit_description.additional_constraints = "Supported values: [none, inch, centimeter]";
  try {
    config_.tiff_res_unit = node->declare_parameter(
      tiff_res_unit_param_name, "inch", tiff_res_unit_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", tiff_res_unit_param_name.c_str());
    config_.tiff_res_unit = node->get_parameter(tiff_res_unit_param_name).get_value<std::string>();
  }

  std::string tiff_xdpi_param_name = param_base_name + ".tiff.xdpi";
  rcl_interfaces::msg::ParameterDescriptor tiff_xdpi_description;
  tiff_xdpi_description.description = "tiff xdpi";
  tiff_xdpi_description.read_only = false;
  try {
    config_.tiff_xdpi = node->declare_parameter(
      tiff_xdpi_param_name, -1, tiff_xdpi_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", tiff_xdpi_param_name.c_str());
    config_.tiff_xdpi = node->get_parameter(tiff_xdpi_param_name).get_value<int64_t>();
  }

  std::string tiff_ydpi_param_name = param_base_name + ".tiff.ydpi";
  rcl_interfaces::msg::ParameterDescriptor tiff_ydpi_description;
  tiff_ydpi_description.description = "tiff ydpi";
  tiff_ydpi_description.read_only = false;
  try {
    config_.tiff_ydpi = node->declare_parameter(
      tiff_ydpi_param_name, -1, tiff_ydpi_description);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", tiff_ydpi_param_name.c_str());
    config_.tiff_ydpi = node->get_parameter(tiff_xdpi_param_name).get_value<int64_t>();
  }
}

void CompressedPublisher::publish(
  const sensor_msgs::msg::Image& message,
  const PublishFn& publish_fn) const
{
  // Compressed image message
  sensor_msgs::msg::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Compression settings
  std::vector<int> params;

  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (config_.format == "jpeg") {
    encodingFormat = JPEG;
  } else if (config_.format == "png") {
    encodingFormat = PNG;
  } else if (config_.format == "tiff") {
    encodingFormat = TIFF;
  }

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  switch (encodingFormat)
  {
    // JPEG Compression
    case JPEG:
    {
      params.reserve(2);
      params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
      params.emplace_back(config_.jpeg_quality);

      // Update ros message format header
      compressed.format += "; jpeg compressed ";

      // Check input format
      if ((bitDepth == 8) || (bitDepth == 16))
      {
        // Target image format
        std::string targetFormat;
        if (enc::isColor(message.encoding))
        {
          // convert color images to BGR8 format
          targetFormat = "bgr8";
          compressed.format += targetFormat;
        } else {
          // convert gray images to mono8 format
          targetFormat = "mono8";
          compressed.format += targetFormat;
        }

        // OpenCV-ros bridge
        try
        {
          std::shared_ptr<CompressedPublisher> tracked_object;
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat);

          // Compress image
          if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
          {

            float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                / (float)compressed.data.size();
            RCLCPP_DEBUG(logger_, "Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
          }
          else
          {
            RCLCPP_ERROR(logger_, "cv::imencode (jpeg) failed on input image");
          }
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(logger_, "%s", e.what());
        }
        catch (cv::Exception& e)
        {
          RCLCPP_ERROR(logger_, "%s", e.what());
        }

        // Publish message
        publish_fn(compressed);
      } else {
        RCLCPP_ERROR(logger_, "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", message.encoding.c_str());
      }
      break;
    }
    // PNG Compression
    case PNG:
    {
      params.reserve(2);
      params.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
      params.emplace_back(config_.png_level);

      // Update ros message format header
      compressed.format += "; png compressed ";

      // Check input format
      if ((bitDepth == 8) || (bitDepth == 16))
      {

        // Target image format
        stringstream targetFormat;
        if (enc::isColor(message.encoding))
        {
          // convert color images to RGB domain
          targetFormat << "bgr" << bitDepth;
          compressed.format += targetFormat.str();
        }

        // OpenCV-ros bridge
        try
        {
          std::shared_ptr<CompressedPublisher> tracked_object;
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat.str());

          // Compress image
          if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
          {

            float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                / (float)compressed.data.size();
            RCUTILS_LOG_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
          }
          else
          {
            RCUTILS_LOG_ERROR("cv::imencode (png) failed on input image");
          }
        }
        catch (cv_bridge::Exception& e)
        {
          RCUTILS_LOG_ERROR("%s", e.what());
          return;
        }
        catch (cv::Exception& e)
        {
          RCUTILS_LOG_ERROR("%s", e.what());
          return;
        }

        // Publish message
        publish_fn(compressed);
      } else {
        RCUTILS_LOG_ERROR(
          "Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)", message.encoding.c_str());
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
      if (config_.tiff_res_unit == "inch") {
        res_unit = 2;
      } else if (config_.tiff_res_unit == "centimeter") {
        res_unit = 3;
      } else if (config_.tiff_res_unit == "none") {
        res_unit = 1;
      } else {
        RCLCPP_WARN(
          logger_,
          "tiff.res_unit parameter should be either 'inch', 'centimeter' or 'none'; "
          "defaulting to 'inch'. Found '%s'", config_.tiff_res_unit.c_str());
      }
      params.reserve(3);
      params.emplace_back(cv::IMWRITE_TIFF_XDPI);
      params.emplace_back(config_.tiff_xdpi);
      params.emplace_back(cv::IMWRITE_TIFF_YDPI);
      params.emplace_back(config_.tiff_ydpi);
      params.emplace_back(cv::IMWRITE_TIFF_RESUNIT);
      params.emplace_back(res_unit);

      // Check input format
      if ((bitDepth == 8) || (bitDepth == 16) || (bitDepth == 32))
      {

        // OpenCV-ros bridge
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, nullptr, "");

          // Compress image
          if (cv::imencode(".tiff", cv_ptr->image, compressed.data, params))
          {
            float cRatio = static_cast<float>((cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()))
                / static_cast<float>((float)compressed.data.size());
            RCUTILS_LOG_DEBUG("Compressed Image Transport - Codec: tiff, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
          }
          else
          {
            RCUTILS_LOG_ERROR("cv::imencode (tiff) failed on input image");
          }
        }
        catch (cv_bridge::Exception& e)
        {
          RCUTILS_LOG_ERROR("%s", e.what());
          return;
        }
        catch (cv::Exception& e)
        {
          RCUTILS_LOG_ERROR("%s", e.what());
          return;
        }

        // Publish message
        publish_fn(compressed);
      } else {
        RCUTILS_LOG_ERROR(
          "Compressed Image Transport - TIFF compression requires 8/16/32-bit encoded color format (input format is: %s)", message.encoding.c_str());
      }
      break;
    }

    default:
      RCUTILS_LOG_ERROR("Unknown compression type '%s', valid options are 'jpeg', 'png' and 'tiff'", config_.format.c_str());
      break;
  }
}
} //namespace compressed_image_transport
