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

void CompressedSubscriber::subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const image_transport::TransportHints& transport_hints)
{
    typedef image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage> Base;
    Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);

    // Set up reconfigure server for this topic
    reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
    ReconfigureServer::CallbackType f = boost::bind(&CompressedSubscriber::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
}


void CompressedSubscriber::configCb(Config& config, uint32_t level)
{
  config_ = config;
  if (config_.mode == "gray") {
      flags_ = CV_LOAD_IMAGE_GRAYSCALE;
  } else if (config_.mode == "gray") {
      flags_ = CV_LOAD_IMAGE_COLOR;
  } else /*if (config_.mode == "unchanged")*/ {
      flags_ = CV_LOAD_IMAGE_UNCHANGED;
  } 
}


void CompressedSubscriber::internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                            const Callback& user_cb)

{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try
  {
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), flags_);

    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    if (split_pos==string::npos)
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
          ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
          break;
      }
    } else
    {
      string image_encoding = message->format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if ( enc::isColor(image_encoding))
      {
        string compressed_encoding = message->format.substr(split_pos);
        bool compressed_bgr_image = (compressed_encoding.find("compressed bgr")!=string::npos);

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
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
    // Publish message to user callback
    user_cb(cv_ptr->toImageMsg());
}

} //namespace compressed_image_transport
