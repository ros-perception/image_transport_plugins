#include "image_transport/simple_subscriber_plugin.h"
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/server.h>
#include <compressed_image_transport/CompressedSubscriberConfig.h>

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
  // Overridden to set up reconfigure server
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
          const Callback& callback, const ros::VoidPtr& tracked_object,
          const image_transport::TransportHints& transport_hints);


  virtual void internalCallback(const sensor_msgs::CompressedImageConstPtr& message,
                                const Callback& user_cb);

  typedef compressed_image_transport::CompressedSubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;
  int flags_;

  void configCb(Config& config, uint32_t level);
};

} //namespace image_transport
