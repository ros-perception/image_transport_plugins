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

#include "compressed_image_transport/compressed_subscriber.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "compressed_image_transport/compression_common.h"

#include <rclcpp/parameter_client.hpp>

#include <limits>
#include <vector>

constexpr const char* kDefaultMode = "unchanged";

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

using CompressedImage = sensor_msgs::msg::CompressedImage;

namespace compressed_image_transport
{

void CompressedSubscriber::subscribeImpl(
    rclcpp::Node * node,
    const std::string& base_topic,
    const Callback& callback,
    rmw_qos_profile_t custom_qos)
{
    logger_ = node->get_logger();
    typedef image_transport::SimpleSubscriberPlugin<CompressedImage> Base;
    Base::subscribeImpl(node, base_topic, callback, custom_qos);
    uint ns_len = node->get_effective_namespace().length();
    std::string param_base_name = base_topic.substr(ns_len);
    std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
    std::string mode_param_name = param_base_name + ".mode";

    std::string mode;
    rcl_interfaces::msg::ParameterDescriptor mode_description;
    mode_description.description = "OpenCV imdecode flags to use";
    mode_description.read_only = false;
    mode_description.additional_constraints = "Supported values: [unchanged, gray, color]";
    try {
      mode = node->declare_parameter(mode_param_name, kDefaultMode, mode_description);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG(logger_, "%s was previously declared", mode_param_name.c_str());
      mode = node->get_parameter(mode_param_name).get_value<std::string>();
    }

    if (mode == "unchanged") {
      config_.imdecode_flag = cv::IMREAD_UNCHANGED;
    } else if (mode == "gray") {
      config_.imdecode_flag = cv::IMREAD_GRAYSCALE;
    } else if (mode == "color") {
      config_.imdecode_flag = cv::IMREAD_COLOR;
    } else {
      RCLCPP_ERROR(logger_, "Unknown mode: %s, defaulting to 'unchanged", mode.c_str());
      config_.imdecode_flag = cv::IMREAD_UNCHANGED;
    }
}


void CompressedSubscriber::internalCallback(const CompressedImage::ConstSharedPtr& message,
                                            const Callback& user_cb)

{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try
  {
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), config_.imdecode_flag);

    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    if (split_pos==std::string::npos)
    {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels())
      {
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
    } else
    {
      std::string image_encoding = message->format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if ( enc::isColor(image_encoding))
      {
        std::string compressed_encoding = message->format.substr(split_pos);
        bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image)
        {
          // if necessary convert colors from bgr to rgb
          if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
        } else
        {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
        }
      }
    }
  }
  catch (cv::Exception& e)
  {
    RCLCPP_ERROR(logger_, "%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
}

} //namespace compressed_image_transport
