#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

namespace compressed_image_transport {

void CompressedSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)
{
  // Decompress
  const CvMat compressed = cvMat(1, message->data.size(), CV_8UC1,
                                 const_cast<unsigned char*>(&message->data[0]));
  cv::WImageBuffer_b decompressed( cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR) );

  // Copy into ROS image message
  boost::shared_ptr<sensor_msgs::Image> image_ptr(new sensor_msgs::Image);
  if ( !sensor_msgs::CvBridge::fromIpltoRosImage(decompressed.Ipl(), *image_ptr) ) {
    ROS_ERROR("Unable to create image message");
    return;
  }
  image_ptr->header = message->header;
  // @todo: don't assume 8-bit channels
  if (decompressed.Channels() == 1) {
    image_ptr->encoding = sensor_msgs::image_encodings::MONO8;
  }
  else if (decompressed.Channels() == 3) {
    image_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  }
  else {
    ROS_ERROR("Unsupported number of channels: %i", decompressed.Channels());
    return;
  }
  
  user_cb(image_ptr);
}

} //namespace compressed_image_transport
