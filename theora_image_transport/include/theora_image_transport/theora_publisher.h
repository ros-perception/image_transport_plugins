#include <image_transport/simple_publisher_plugin.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <dynamic_reconfigure/server.h>
#include <theora_image_transport/TheoraPublisherConfig.h>
#include <theora_image_transport/Packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraPublisher : public image_transport::SimplePublisherPlugin<theora_image_transport::Packet>
{
public:
  TheoraPublisher();

  ~TheoraPublisher();
  
  // Return the system unique string representing the theora transport type
  virtual std::string getTransportName() const { return "theora"; }

protected:
  // Overridden to tweak arguments and set up reconfigure server
  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const image_transport::SubscriberStatusCallback  &user_connect_cb,
                             const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                             const ros::VoidPtr &tracked_object, bool latch);
  
  // Callback to send header packets to new clients
  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub);

  // Main publish function
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;

  // Dynamic reconfigure support
  typedef theora_image_transport::TheoraPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  void configCb(Config& config, uint32_t level);

  // Utility functions
  bool ensureEncodingContext(const sensor_msgs::Image& image, const PublishFn& publish_fn) const;
  void oggPacketToMsg(const std_msgs::Header& header, const ogg_packet &oggpacket,
                      theora_image_transport::Packet &msg) const;
  void updateKeyframeFrequency() const;

  // Some data is preserved across calls to publish(), but from the user's perspective publish() is
  // "logically const"
  mutable cv_bridge::CvImage img_image_;
  mutable th_info encoder_setup_;
  mutable ogg_uint32_t keyframe_frequency_;
  mutable boost::shared_ptr<th_enc_ctx> encoding_context_;
  mutable std::vector<theora_image_transport::Packet> stream_header_;
};

} //namespace compressed_image_transport
