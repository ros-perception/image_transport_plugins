#include "image_transport/simple_publisher_plugin.h"
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/server.h>
#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>

namespace compressed_depth_image_transport {

class CompressedDepthPublisher : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
  virtual ~CompressedDepthPublisher() {}

  virtual std::string getTransportName() const
  {
    return "compressedDepth";
  }

protected:
  // Overridden to set up reconfigure server
  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const image_transport::SubscriberStatusCallback  &user_connect_cb,
                             const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                             const ros::VoidPtr &tracked_object, bool latch);
  
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;

  typedef compressed_depth_image_transport::CompressedDepthPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void configCb(Config& config, uint32_t level);
};

} //namespace compressed_depth_image_transport
