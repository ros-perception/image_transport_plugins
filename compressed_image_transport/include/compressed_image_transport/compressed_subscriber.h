#include "image_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>

namespace compressed_image_transport {

class CompressedSubscriber : public image_transport::SimpleSubscriberPlugin<sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  virtual void internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                const Callback& user_cb);
};

} //namespace image_transport
