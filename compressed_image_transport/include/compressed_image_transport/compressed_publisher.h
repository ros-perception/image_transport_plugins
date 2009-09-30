#include "image_transport/simple_publisher_plugin.h"
#include <sensor_msgs/CompressedImage.h>

namespace compressed_image_transport {

class CompressedPublisher : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedPublisher() {}

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;
};

} //namespace compressed_image_transport
