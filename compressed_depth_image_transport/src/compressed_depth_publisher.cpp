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

#include "compressed_depth_image_transport/compressed_depth_publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "compressed_depth_image_transport/codec.h"
#include "compressed_depth_image_transport/compression_common.h"

#include <rclcpp/parameter_client.hpp>

#include <vector>
#include <sstream>

constexpr int kDefaultPngLevel = 9;
constexpr double kDefaultDepthMax = 10.0;
constexpr double KDefaultDepthQuantization = 100.0;

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_depth_image_transport
{

void CompressedDepthPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string& base_topic,
  rmw_qos_profile_t custom_qos)
{
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos);

  node->get_parameter_or<int>("png_level", config_.png_level, kDefaultPngLevel);
  node->get_parameter_or<double>("depth_max", config_.depth_max, kDefaultDepthMax);
  node->get_parameter_or<double>("depth_quantization", config_.depth_max, KDefaultDepthQuantization);
}

void CompressedDepthPublisher::publish(
  const sensor_msgs::msg::Image& message,
  const PublishFn& publish_fn) const
{
  sensor_msgs::msg::CompressedImage::SharedPtr compressed_image =
    encodeCompressedDepthImage(message,
                               config_.depth_max,
                               config_.depth_quantization,
                               config_.png_level);
  if (compressed_image)
  {
    publish_fn(*compressed_image);
  }
}

} //namespace compressed_depth_image_transport
