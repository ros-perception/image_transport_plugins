#include "compressed_image_transport/compressed_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/highgui.h>
#include <boost/make_shared.hpp>

namespace compressed_image_transport {

void CompressedPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                        const image_transport::SubscriberStatusCallback  &user_connect_cb,
                                        const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
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

void CompressedPublisher::publish(const sensor_msgs::Image& message,
                                  const PublishFn& publish_fn) const
{
  // View/convert as mono or RGB
  sensor_msgs::CvBridge bridge;
  /// @todo This probably misses some cases
  /// @todo What about bayer??
  /// @todo Try to avoid deprecated fromImage
  if (bridge.encoding_as_fmt(message.encoding) == "GRAY") {
    if (!bridge.fromImage(message, sensor_msgs::image_encodings::MONO8)) {
      ROS_ERROR("Could not convert image from %s to mono8", message.encoding.c_str());
      return;
    }
  }
  else if (!bridge.fromImage(message, sensor_msgs::image_encodings::BGR8)) {
    ROS_ERROR("Could not convert image from %s to bgr8", message.encoding.c_str());
    return;
  }

  // Fill compression settings
  int params[3] = {0};
  if (config_.format == "jpeg") {
    params[0] = CV_IMWRITE_JPEG_QUALITY;
    params[1] = config_.jpeg_quality;
  }
  else if (config_.format == "png") {
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = config_.png_level;
  }
  else {
    ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'",
              config_.format.c_str());
    return;
  }
  std::string extension = '.' + config_.format;

  // Compress image
  const IplImage* image = bridge.toIpl();
  /// @todo Use cv::imencode, can write directly to compressed.data
  CvMat* buf = cvEncodeImage(extension.c_str(), image, params);

  // Set up message and publish
  sensor_msgs::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = config_.format;
  compressed.data.resize(buf->width);
  memcpy(&compressed.data[0], buf->data.ptr, buf->width);
  cvReleaseMat(&buf);
  
  publish_fn(compressed);
}

} //namespace compressed_image_transport
