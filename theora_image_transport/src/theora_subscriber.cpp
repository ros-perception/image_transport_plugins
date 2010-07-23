#include "theora_image_transport/theora_subscriber.h"
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <boost/scoped_array.hpp>
#include <vector>

using namespace std;

namespace theora_image_transport {

TheoraSubscriber::TheoraSubscriber()
  : received_header_(false),
    decoding_context_(NULL),
    setup_info_(NULL)
{
  th_info_init(&header_info_);
  th_comment_init(&header_comment_);
  headers_received_ = 0; // DEBUG
}

TheoraSubscriber::~TheoraSubscriber()
{
  if (decoding_context_) th_decode_free(decoding_context_);
  th_setup_free(setup_info_);
  th_info_clear(&header_info_);
  th_comment_clear(&header_comment_);
}

void TheoraSubscriber::subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                     const Callback &callback, const ros::VoidPtr &tracked_object,
                                     const image_transport::TransportHints &transport_hints)
{
  // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
  queue_size += 4;
  typedef image_transport::SimpleSubscriberPlugin<theora_image_transport::Packet> Base;
  Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
}

//When using this caller is responsible for deleting oggpacket.packet!!
void TheoraSubscriber::msgToOggPacket(const theora_image_transport::Packet &msg, ogg_packet &ogg)
{
  ogg.bytes      = msg.data.size();
  ogg.b_o_s      = msg.b_o_s;
  ogg.e_o_s      = msg.e_o_s;
  ogg.granulepos = msg.granulepos;
  ogg.packetno   = msg.packetno;
  ogg.packet = new unsigned char[ogg.bytes];
  memcpy(ogg.packet, &msg.data[0], ogg.bytes);
}

void TheoraSubscriber::internalCallback(const theora_image_transport::PacketConstPtr& message, const Callback& callback)
{
  /// @todo Break this function into pieces
  ogg_packet oggpacket;
  msgToOggPacket(*message, oggpacket);
  boost::scoped_array<unsigned char> packet_guard(oggpacket.packet); // Make sure packet memory gets deleted

  // Beginning of logical stream flag means we're getting new headers
  if (oggpacket.b_o_s == 1) {
    received_header_ = false;
    if (decoding_context_) {
      th_decode_free(decoding_context_);
      decoding_context_ = NULL;
    }
    th_setup_free(setup_info_);
    setup_info_ = NULL;
  }

  // Decode header packets until we get the first video packet
  if (received_header_ == false) {
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    switch (rval) {
      case 0:
        // We've received the full header; this is the first video packet.
        ROS_INFO("Full header received! Got %d header packets", headers_received_);
        decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
        if (!decoding_context_) {
          ROS_ERROR("[theora] Decoding parameters were invalid");
          return;
        }
        received_header_ = true;
        break; // Continue on the video decoding
      case TH_EFAULT:
        ROS_WARN("[theora] EFAULT when processing header packet");
        return;
      case TH_EBADHEADER:
        ROS_WARN("[theora] Bad header packet");
        return;
      case TH_EVERSION:
        ROS_WARN("[theora] Header packet not decodable with this version of libtheora");
        return;
      case TH_ENOTFORMAT:
        ROS_WARN("[theora] Packet was not a Theora header");
        return;
      default:
        // If rval > 0, we successfully received a header packet.
        if (rval < 0)
          ROS_WARN("[theora] Error code %d when processing header packet", rval);
        else {
          // Successfully decoded a header packet
          ROS_INFO("Successfully received a header packet");
          headers_received_++; // DEBUG
        }
        return;
    }
  }

  /// @todo Wait for a keyframe if we haven't received one yet
  // We have a video packet, let's decode it
  int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);
  switch (rval) {
    case 0:
      break; // Yay, we got a frame. Carry on.
    case TH_DUPFRAME:
      // Video data hasn't changed, so we reuse the last received frame.
      ROS_INFO("[theora] Got a duplicate frame"); /// @todo Change to debug
      if (latest_image_) {
        latest_image_->header = message->header;
        callback(latest_image_);
      }
      return;
    case TH_EFAULT:
      ROS_WARN("[theora] EFAULT processing video packet");
      return;
    case TH_EBADPACKET:
      ROS_WARN("[theora] Packet does not contain encoded video data");
      return;
    case TH_EIMPL:
      ROS_WARN("[theora] The video data uses bitstream features not supported by this version of libtheora");
      return;
    default:
      ROS_WARN("[theora] Error code %d when decoding video packet", rval);
      return;
  }

  // We have a new decoded frame available
  th_ycbcr_buffer ycbcr_buffer;
  th_decode_ycbcr_out(decoding_context_, ycbcr_buffer);

  // Wrap YCbCr channel data into OpenCV format
  th_img_plane &y_plane = ycbcr_buffer[0], &cb_plane = ycbcr_buffer[1], &cr_plane = ycbcr_buffer[2];
  cv::Mat y(y_plane.height, y_plane.width, CV_8UC1, y_plane.data, y_plane.stride);
  cv::Mat cb_sub(cb_plane.height, cb_plane.width, CV_8UC1, cb_plane.data, cb_plane.stride);
  cv::Mat cr_sub(cr_plane.height, cr_plane.width, CV_8UC1, cr_plane.data, cr_plane.stride);

  // Upsample chroma channels
  cv::Mat cb, cr;
  cv::pyrUp(cb_sub, cb);
  cv::pyrUp(cr_sub, cr);

  // Merge into interleaved image. Note OpenCV uses YCrCb, so we swap the chroma channels.
  cv::Mat ycrcb, channels[] = {y, cr, cb};
  cv::merge(channels, 3, ycrcb);

  // Convert to BGR color
  cv::Mat bgr, bgr_padded;
  cv::cvtColor(ycrcb, bgr_padded, CV_YCrCb2BGR);
  // Pull out original (non-padded) image region
  bgr = bgr_padded(cv::Rect(header_info_.pic_x, header_info_.pic_y,
                            header_info_.pic_width, header_info_.pic_height));

  IplImage ipl = bgr;
  latest_image_ = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
  latest_image_->header = message->header;
  /// @todo Copy connection header (also in DUPFRAME above)
  /// @todo Handle RGB8 or MONO8 efficiently
  latest_image_->encoding = sensor_msgs::image_encodings::BGR8;
  callback(latest_image_);
}

} //namespace theora_image_transport
