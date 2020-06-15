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
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/make_shared.hpp>

#include "compressed_image_transport/compression_common.h"

#include <vector>
#include <sstream>

// If OpenCV4
#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

void CompressedPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                        const image_transport::SubscriberStatusCallback &user_connect_cb,
                                        const image_transport::SubscriberStatusCallback &user_disconnect_cb,
                                        const ros::VoidPtr &tracked_object, bool latch)
{
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f = boost::bind(&CompressedPublisher::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void CompressedPublisher::configCb(Config& config, uint32_t level)
{
  config_ = config;
}

void CompressedPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (config_.format == compressed_image_transport::CompressedPublisher_jpeg)
    encodingFormat = JPEG;
  if (config_.format == compressed_image_transport::CompressedPublisher_png)
    encodingFormat = PNG;

  // Compression settings
  std::vector<int> params;
  sensor_msgs::CompressedImagePtr compressed;
  switch (encodingFormat)
  {
    // JPEG Compression
    case JPEG:
    {
      params.resize(8, 0);
      params[0] = IMWRITE_JPEG_QUALITY;
      params[1] = config_.jpeg_quality;
      params[2] = IMWRITE_JPEG_PROGRESSIVE;
      params[3] = config_.jpeg_progressive ? 1 : 0;
      params[4] = IMWRITE_JPEG_OPTIMIZE;
      params[5] = config_.jpeg_optimize ? 1 : 0;
      params[6] = IMWRITE_JPEG_RST_INTERVAL;
      params[7] = config_.jpeg_restart_interval;

      // Publish message
      try {
        compressed = encodeImage(message, encodingFormat, params);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("%s", e.what());
      }
      catch (cv::Exception& e) {
        ROS_ERROR("%s", e.what());
      }
      catch (std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
      }
      break;
    }

    // PNG Compression
    case PNG:
    {
      params.resize(2, 0);
      params[0] = IMWRITE_PNG_COMPRESSION;
      params[1] = config_.png_level;
      try { 
        compressed = encodeImage(message, encodingFormat, params);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("%s", e.what());
      }
      catch (cv::Exception& e) {
        ROS_ERROR("%s", e.what());
      }
      catch (std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
      }
      break;
    }
    default:
      ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
      break;
  }
  // Publish message
  publish_fn(*compressed);

}

} //namespace compressed_image_transport
