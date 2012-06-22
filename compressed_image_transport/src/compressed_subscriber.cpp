#include "compressed_image_transport/compressed_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "compressed_image_transport/compression_common.h"

#include <limits>
#include <vector>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport
{

void CompressedSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)

{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Assign image encoding
  string image_encoding = message->format.substr(0, message->format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode color/mono image
  try
  {
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), CV_LOAD_IMAGE_UNCHANGED);

    if (enc::isColor(image_encoding))
    {
      // Revert color transformation
      if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

      if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

      if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
    }
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
}

} //namespace compressed_image_transport
