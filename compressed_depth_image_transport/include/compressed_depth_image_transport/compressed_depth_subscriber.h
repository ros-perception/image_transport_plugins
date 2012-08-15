#include "image_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>

namespace compressed_depth_image_transport {

class CompressedDepthSubscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedDepthSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "compressedDepth";
  }

protected:
  virtual void internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                const Callback& user_cb);
};

} //namespace compressed_depth_image_transport
