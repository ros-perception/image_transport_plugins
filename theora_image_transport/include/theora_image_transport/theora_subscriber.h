#include <image_transport/simple_subscriber_plugin.h>
#include <dynamic_reconfigure/server.h>
#include <theora_image_transport/TheoraSubscriberConfig.h>
#include <theora_image_transport/Packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraSubscriber : public image_transport::SimpleSubscriberPlugin<theora_image_transport::Packet>
{
public:
  TheoraSubscriber();
  virtual ~TheoraSubscriber();

  virtual std::string getTransportName() const { return "theora"; }

protected:
  // Overridden to bump queue_size, otherwise we might lose headers
  // Overridden to tweak arguments and set up reconfigure server
  virtual void subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                             const Callback &callback, const ros::VoidPtr &tracked_object,
                             const image_transport::TransportHints &transport_hints);
  
  // The function that does the actual decompression and calls a user supplied callback with the resulting image
  virtual void internalCallback(const theora_image_transport::PacketConstPtr &msg, const Callback& user_cb);

  // Dynamic reconfigure support
  typedef theora_image_transport::TheoraSubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  int pplevel_; // Post-processing level

  void configCb(Config& config, uint32_t level);

  // Utility functions
  int updatePostProcessingLevel(int level);
  void msgToOggPacket(const theora_image_transport::Packet &msg, ogg_packet &ogg);

  bool received_header_;
  bool received_keyframe_;
  th_dec_ctx* decoding_context_;
  th_info header_info_;
  th_comment header_comment_;
  th_setup_info* setup_info_;
  sensor_msgs::ImagePtr latest_image_;
};

} //namespace theora_image_transport
