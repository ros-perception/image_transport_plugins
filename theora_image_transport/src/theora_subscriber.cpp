#include "theora_image_transport/theora_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <vector>

#define null 0

using namespace std;

namespace theora_image_transport {

TheoraSubscriber::TheoraSubscriber()
{
  decoding_context_ = null;
  received_header_ = false;
  setup_info_ = null;
  th_info_init(&header_info_);
}

TheoraSubscriber::~TheoraSubscriber()
{
}

//When using this caller is responsible for deleting oggpacket.packet!!
void TheoraSubscriber::msgToOggPacket(const theora_image_transport::packet &msg, ogg_packet &oggpacketOutput)
{
  oggpacketOutput.bytes = msg.bytes;
  oggpacketOutput.b_o_s = msg.b_o_s;
  oggpacketOutput.e_o_s = msg.e_o_s;
  oggpacketOutput.granulepos = msg.granulepos;
  oggpacketOutput.packetno = msg.packetno;
  oggpacketOutput.packet = new unsigned char[msg.bytes];
  memcpy(oggpacketOutput.packet, &msg.blob[0], msg.bytes);

  //ROS_DEBUG("Received %d bytes in packet#%d and granule%d (and this is BOS: %d).", oggpacketOutput.bytes, oggpacketOutput.packetno, oggpacketOutput.granulepos, oggpacketOutput.b_o_s);
  /*unsigned int i = 0;
  for (int j = 0; j < msg.bytes; j++)
    i = i * 2 % 91 + oggpacketOutput.packet[j];
  ROS_DEBUG("Checksum is: %d", i);*/
}

void TheoraSubscriber::internalCallback(const theora_image_transport::packetConstPtr& message, const Callback& callback)
{
  ogg_packet oggpacket;
  msgToOggPacket(*message, oggpacket); /// @todo smart ptr around oggpacket.packet?
  sensor_msgs::Image *rosMsg = null; /// @todo This is not safe

  if (received_header_ == false) //still receiving header info
  {
    if ((int)oggpacket.packetno == 999999)
    {
      ROS_DEBUG("Dropping flush packet.");
      return;
    }

    /*if(oggpacket.packetno != 0)
     {
     ROS_DEBUG("Dumping header packet because packet# is non-zero: %d.", (int)oggpacket.packetno);
     return;
     }*/
    //static th_setup_info* setup_info_ptr = null;
    //static th_setup_info** setup_info_ = &setup_info_ptr;
    ROS_DEBUG("Setup_info: %p", setup_info_);
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    ROS_DEBUG("Setup_info: %p", setup_info_);
    if (rval == 0)
    {
      ROS_DEBUG("This should happen on correct receipt of a header packet but never seems to in practice");
      //received_header_ = true;
      //decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    }
    else if (rval == TH_EFAULT)
      ROS_DEBUG("EFault when processing header.");
    else if (rval == TH_EBADHEADER) //Oddly, I seem to always get one of these...
      ROS_DEBUG("Bad header when processing header.");
    else if (rval == TH_EVERSION)
      ROS_DEBUG("Bad version when processing header.");
    else if (rval == TH_ENOTFORMAT)
      ROS_DEBUG("Received packet which was not a Theora header.");
    else if (rval < 0)
      ROS_DEBUG("Error code when processing header: %d.", rval);

    /// @todo This seems suspicious, the docs are pretty clear
    if (setup_info_ != null)  //Because rval != 0 as specified in the docs, this is used instead to check that
    {                         // the header has been fully received
      received_header_ = true;
      decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    }
  }

  if (received_header_ == true)
  {
    int rval = th_decode_packetin(decoding_context_, &oggpacket, null);

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
      rosMsg = new sensor_msgs::Image;
      img_bridge_.fromIpltoRosImage(&ipl, *rosMsg);
    }
    else if (rval == TH_DUPFRAME)
      ROS_DEBUG("Got a duplicate frame.");
    else
      ROS_DEBUG("Error code when decoding packet: %d.", rval);
  }

  delete oggpacket.packet;

  if(rosMsg != null)
  {
    //The shared pointer will take care of freeing rosMsg
    boost::shared_ptr<sensor_msgs::Image> image_ptr(rosMsg);

    //Manually set encoding to be correct
    //TODO: the packet message could be extended with a flag that indicates the original type for better handling of
    //      B&W images
    image_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    callback(image_ptr);
  }
}

} //namespace theora_image_transport
