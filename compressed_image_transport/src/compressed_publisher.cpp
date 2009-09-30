#include "compressed_image_transport/compressed_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv_latest/CvBridge.h>
#include <opencv/highgui.h>

namespace compressed_image_transport {

void CompressedPublisher::publish(const sensor_msgs::Image& message,
                                  const PublishFn& publish_fn) const
{
  // View/convert as mono or RGB
  sensor_msgs::CvBridge bridge;
  // @todo: this probably misses some cases
  // @todo: what about bayer??
  if (bridge.encoding_as_fmt(message.encoding) == "GRAY") {
    if (!bridge.fromImage(message, sensor_msgs::image_encodings::MONO8)) {
      ROS_ERROR("Could not convert image from %s to mono8", message.encoding.c_str());
      return;
    }
  }
  else if (!bridge.fromImage(message, sensor_msgs::image_encodings::RGB8)) {
    ROS_ERROR("Could not convert image from %s to rgb8", message.encoding.c_str());
    return;
  }

  // Update settings from parameter server
  int params[3] = {0};
  std::string format;
  if (!nh().getParam("compression_type", format, true))
    format = "jpeg";
  if (format == "jpeg") {
    params[0] = CV_IMWRITE_JPEG_QUALITY;
    params[1] = 80; // default: 80% quality
  }
  else if (format == "png") {
    params[0] = CV_IMWRITE_PNG_COMPRESSION;
    params[1] = 9; // default: maximum compression
  }
  else {
    ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'",
              format.c_str());
    return;
  }
  nh().getParam("compression_level", params[1], true);
  std::string extension = '.' + format;

  // Compress image
  const IplImage* image = bridge.toIpl();
  CvMat* buf = cvEncodeImage(extension.c_str(), image, params);

  // Set up message and publish
  sensor_msgs::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = format;
  compressed.data.resize(buf->width);
  memcpy(&compressed.data[0], buf->data.ptr, buf->width);
  cvReleaseMat(&buf);
  
  publish_fn(compressed);
}

} //namespace compressed_image_transport
