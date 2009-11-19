#include "image_transport/simple_subscriber_plugin.h"
#include <theora_image_transport/packet.h>
#include <cv_bridge/CvBridge.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraSubscriber : public image_transport::SimpleSubscriberPlugin<theora_image_transport::packet>
{
public:
  TheoraSubscriber();
  virtual ~TheoraSubscriber();

  virtual std::string getTransportName() const
  {
    return "theora";
  }

protected:
  //The function that does the actual decompression and calls a user supplied callback with the resulting image
  virtual void internalCallback(const theora_image_transport::packet::ConstPtr &msg, const Callback& user_cb);

private:
  void msgToOggPacket(const theora_image_transport::packet &msg, ogg_packet &oggpacketOutput);

  bool received_header_;
  th_dec_ctx* decoding_context_;
  th_info header_info_;
  th_comment header_comment_;
  th_setup_info* setup_info_;
  sensor_msgs::CvBridge img_bridge_;
};

} //namespace theora_image_transport
