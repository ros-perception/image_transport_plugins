#include "theora_image_transport/theora_subscriber.h"
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <vector>

using namespace std;

namespace theora_image_transport {

TheoraSubscriber::TheoraSubscriber()
{
  decoding_context_ = NULL;
  received_header_ = false;
  setup_info_ = NULL;
  th_info_init(&header_info_);
  th_comment_init(&header_comment_);
  headers_received_ = 0;
}

TheoraSubscriber::~TheoraSubscriber()
{
  th_info_clear(&header_info_);
  th_comment_clear(&header_comment_);
  /// @todo decoding_context_, setup_info_
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
  ogg_packet oggpacket;
  msgToOggPacket(*message, oggpacket); /// @todo smart ptr around oggpacket.packet?
  sensor_msgs::ImagePtr image_ptr;

  if (received_header_ == false) //still receiving header info
  {
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    if (rval > 0) {
      ROS_INFO("Successfully received a header packet");
      headers_received_++;
    }
    if (rval == 0)
    {
      ROS_INFO("Full header received! Got %d header packets", headers_received_);
      received_header_ = true;
      decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    }
    else if (rval == TH_EFAULT)
      ROS_WARN("EFault when processing header.");
    else if (rval == TH_EBADHEADER) //Oddly, I seem to always get one of these...
      ROS_WARN("Bad header packet.");
    else if (rval == TH_EVERSION)
      ROS_WARN("Bad version when processing header.");
    else if (rval == TH_ENOTFORMAT)
      ROS_WARN("Received packet which was not a Theora header.");
    else if (rval < 0)
      ROS_WARN("Error code when processing header: %d.", rval);
  }

  if (received_header_ == true)
  {
    int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);

    if (rval == 0) //Successfully got a frame
    {
      th_ycbcr_buffer ycbcr_buffer;
      th_decode_ycbcr_out(decoding_context_, ycbcr_buffer);

      // Blah
      th_img_plane &y_plane = ycbcr_buffer[0], &cb_plane = ycbcr_buffer[1], &cr_plane = ycbcr_buffer[2];
      cv::Mat y(y_plane.height, y_plane.width, CV_8UC1, y_plane.data, y_plane.stride);
      cv::Mat cb_sub(cb_plane.height, cb_plane.width, CV_8UC1, cb_plane.data, cb_plane.stride);
      cv::Mat cr_sub(cr_plane.height, cr_plane.width, CV_8UC1, cr_plane.data, cr_plane.stride);

      // Upsample chroma channels
      cv::Mat cb, cr;
      cv::pyrUp(cb_sub, cb);
      cv::pyrUp(cr_sub, cr);

      // Merge into interleaved image
      cv::Mat y2 = y.clone();
      cv::Mat ycrcb, channels[] = {y2, cr, cb};
      cv::merge(channels, 3, ycrcb);

      // Convert to BGR color
      cv::Mat bgr, bgr_padded;
      cv::cvtColor(ycrcb, bgr_padded, CV_YCrCb2BGR);
      // Pull out original (non-padded) image region
      bgr = bgr_padded(cv::Rect(header_info_.pic_x, header_info_.pic_y,
                                header_info_.pic_width, header_info_.pic_height));

      //ROS_INFO("Decoded image %d x %d", bgr.cols, bgr.rows);
      IplImage ipl = bgr;
      image_ptr = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
    }
    else if (rval == TH_DUPFRAME)
      ROS_INFO("Got a duplicate frame.");
    else if (rval == TH_EFAULT)
      ROS_WARN("EFAULT processing packet.");
    else if (rval == TH_EBADPACKET)
      ROS_WARN("Packet does not contain encoded video data.");
    else if (rval == TH_EIMPL)
      ROS_WARN("The video data uses bitstream features not supported by this version of libtheora.");
    else
      ROS_WARN("Error code when decoding packet: %d.", rval);
  }

  delete oggpacket.packet;

  if (image_ptr) {
    image_ptr->header = message->header;
    //Manually set encoding to be correct
    //TODO: the packet message could be extended with a flag that indicates the original type for better handling of
    //      B&W images
    image_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    callback(image_ptr);
  }
}

} //namespace theora_image_transport
