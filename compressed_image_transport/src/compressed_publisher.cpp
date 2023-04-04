/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 20012, Willow Garage, Inc.
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
#include <rclcpp/parameter_events_filter.hpp>

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

  declareParameters(node, base_topic);
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
  params.resize(3, 0);

  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (config_.format == "jpeg")
    encodingFormat = JPEG;
  if (config_.format == "png")
    encodingFormat = PNG;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);

  switch (encodingFormat)
  {
    // JPEG Compression
    case JPEG:
    {
      params[0] = cv::IMWRITE_JPEG_QUALITY;
      params[1] = config_.jpeg_quality;

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
      }
      else
      {
        RCLCPP_ERROR(logger_, "Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)", message.encoding.c_str());
      }
      break;
    }
      // PNG Compression
    case PNG:
    {
      params[0] = cv::IMWRITE_PNG_COMPRESSION;
      params[1] = config_.png_level;

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
      }
      else
        RCUTILS_LOG_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)", message.encoding.c_str());
      break;
    }

    default:
      RCUTILS_LOG_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
      break;
  }
}

void CompressedPublisher::declareParameters(rclcpp::Node* node, const std::string& base_topic)
{
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  const std::string transport_name = getTransportName();

  using callbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto callback = std::bind(&CompressedPublisher::onParameterEvent, this, std::placeholders::_1, node->get_fully_qualified_name());

  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event<callbackT>(node, callback);

  config_.format = declareParameter(node, param_base_name, transport_name, rclcpp::ParameterValue(kDefaultFormat),
                                    rcl_interfaces::msg::ParameterDescriptor()
                                      .set__name("format")
                                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                                      .set__description("Compression method")
                                      .set__read_only(false)
                                      .set__additional_constraints("Supported values: [jpeg, png]")).get<rclcpp::ParameterType::PARAMETER_STRING>();

  config_.png_level = declareParameter(node, param_base_name, transport_name, rclcpp::ParameterValue(kDefaultPngLevel),
                                       rcl_interfaces::msg::ParameterDescriptor()
                                        .set__name("png_level")
                                        .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                        .set__description("Compression level for PNG format")
                                        .set__read_only(false)
                                        .set__integer_range(
                                          {rcl_interfaces::msg::IntegerRange()
                                            .set__from_value(0)
                                            .set__to_value(9)
                                            .set__step(1)})).get<rclcpp::ParameterType::PARAMETER_INTEGER>();

  config_.jpeg_quality = declareParameter(node, param_base_name, transport_name, rclcpp::ParameterValue(kDefaultJpegQuality),
                                          rcl_interfaces::msg::ParameterDescriptor()
                                            .set__name("jpeg_quality")
                                            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                            .set__description("Image quality for JPEG format")
                                            .set__read_only(false)
                                            .set__integer_range(
                                              {rcl_interfaces::msg::IntegerRange()
                                                .set__from_value(1)
                                                .set__to_value(100)
                                                .set__step(1)})).get<rclcpp::ParameterType::PARAMETER_INTEGER>();
}

rclcpp::ParameterValue CompressedPublisher::declareParameter(rclcpp::Node* node,
                                                             const std::string &base_name,
                                                             const std::string &transport_name,
                                                             const rclcpp::ParameterValue &default_value,
                                                             const rcl_interfaces::msg::ParameterDescriptor &descriptor)
{
  rclcpp::ParameterValue param_value;

  //transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string param_name = base_name + "." + transport_name + "." + descriptor.name;

  try {
    param_value = node->declare_parameter(param_name, default_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", descriptor.name.c_str());
    param_value = node->get_parameter(param_name).get_parameter_value();
  }

  rclcpp::ParameterValue value_set = param_value;

  //deprecated non-scoped parameter name (e.g. image_raw.format)
  const std::string deprecated_name = base_name + "." + descriptor.name;
  deprecatedParameters_.push_back(deprecated_name);

  // transport scoped parameter as default, otherwise we would overwrite
  try {
    param_value = node->declare_parameter(deprecated_name, param_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", descriptor.name.c_str());
    param_value = node->get_parameter(deprecated_name).get_parameter_value();
  }

  // in case parameter was set through deprecated keep transport scoped parameter in sync
  if(value_set != param_value)
    node->set_parameter(rclcpp::Parameter(param_name, param_value));

  return param_value;
}

void CompressedPublisher::onParameterEvent(ParameterEvent::SharedPtr event, std::string full_name)
{
  // filter out events from other nodes
  if (event->node != full_name)
    return;

  // filter out new/changed deprecated parameters
  using EventType = rclcpp::ParameterEventsFilter::EventType;

  rclcpp::ParameterEventsFilter filter(event, deprecatedParameters_, {EventType::NEW, EventType::CHANGED});

  if(!filter.get_events().size())
    return;

  // emit warnings for deprecated parameters
  const std::string transport = getTransportName();

  for (auto & it : filter.get_events())
  {
    const std::string name = it.second->name;
    size_t dotIndex = name.find_last_of('.');
    std::string recommended = name.substr(0, dotIndex + 1) + transport + name.substr(dotIndex);

    RCLCPP_WARN_STREAM(logger_, "parameter `" << name << "` is deprecated" <<
                                "; use transport qualified name `" << recommended << "`");
  }
}

} //namespace compressed_image_transport
