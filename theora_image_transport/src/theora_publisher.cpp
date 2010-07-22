#include "theora_image_transport/theora_publisher.h"
#include <sensor_msgs/image_encodings.h>

#include <vector>
#include <cstdio> //for memcpy

using namespace std;

namespace theora_image_transport {

TheoraPublisher::TheoraPublisher()
{
  // Initialize info structure fields that don't change
  th_info_init(&encoder_setup_);
  
  encoder_setup_.pic_x = 0;
  encoder_setup_.pic_y = 0;
  encoder_setup_.colorspace = TH_CS_UNSPECIFIED;
  encoder_setup_.pixel_fmt = TH_PF_420; // See bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
  /// @todo Make target bitrate and quality parameters
  //int bitrate;
  //nh().param("theora_bitrate", bitrate, 800000);
  encoder_setup_.target_bitrate = 800000;
  //encoder_setup_.quality = 63;    // On a scale of 0 to 63, to use this set target bitrate to 0
  encoder_setup_.aspect_numerator = 1;
  encoder_setup_.aspect_denominator = 1;
  encoder_setup_.fps_numerator = 0;
  encoder_setup_.fps_denominator = 0;
  encoder_setup_.keyframe_granule_shift = 6; // Apparently a good default
}

TheoraPublisher::~TheoraPublisher()
{
  th_info_clear(&encoder_setup_);
}

void TheoraPublisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                    const image_transport::SubscriberStatusCallback  &user_connect_cb,
                                    const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                                    const ros::VoidPtr &tracked_object, bool latch)
{
  // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
  queue_size += 4;
  typedef image_transport::SimplePublisherPlugin<theora_image_transport::Packet> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);
  /// @todo Take over latch, and save the most recent keyframe
}

// Sends the header packets to new subscribers
void TheoraPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  ROS_INFO("In connectCallback, publishing %d header packets", (int)stream_header_.size());
  for (unsigned int i = 0; i < stream_header_.size(); i++) {
    pub.publish(stream_header_[i]);
  }
}

static void cvToTheoraPlane(cv::Mat& mat, th_img_plane& plane)
{
  plane.width  = mat.cols;
  plane.height = mat.rows;
  plane.stride = mat.step;
  plane.data   = mat.data;
}

void TheoraPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  /// @todo fromImage is deprecated
  /// @todo Optimized gray-scale path, rgb8
  if (!img_bridge_.fromImage(message, "bgr8")) {
    ROS_ERROR("Unable to convert from '%s' to bgr8", message.encoding.c_str());
    return;
  }

  cv::Mat bgr(img_bridge_.toIpl()), bgr_padded;
  ensure_encoding_context(message, publish_fn);
  // Pad dimensions out to multiples of 16
  int frame_width = encoder_setup_.frame_width, frame_height = encoder_setup_.frame_height;
  if (frame_width == bgr.cols && frame_height == bgr.rows) {
    bgr_padded = bgr;
  }
  else {
    bgr_padded = cv::Mat::zeros(frame_height, frame_width, bgr.type());
    cv::Mat pic_roi = bgr_padded(cv::Rect(0, 0, bgr.cols, bgr.rows));
    bgr.copyTo(pic_roi);
  }

  //convert image
  cv::Mat ycrcb;
  cv::cvtColor(bgr_padded, ycrcb, CV_BGR2YCrCb);
  // Split channels
  cv::Mat ycrcb_planes[3];
  cv::split(ycrcb, ycrcb_planes);

  // Use Y as-is but subsample chroma channels
  cv::Mat y = ycrcb_planes[0], cr, cb;
  cv::pyrDown(ycrcb_planes[1], cr);
  cv::pyrDown(ycrcb_planes[2], cb);

  // Construct Theora image buffer
  th_ycbcr_buffer ycbcr_buffer;
  cvToTheoraPlane(y,  ycbcr_buffer[0]);
  cvToTheoraPlane(cb, ycbcr_buffer[1]);
  cvToTheoraPlane(cr, ycbcr_buffer[2]);

  /// @todo Louder errors?
  int rval;
  if (!encoding_context_)
    ROS_WARN("About to encode with null encoding context.");
  rval = th_encode_ycbcr_in(encoding_context_.get(), ycbcr_buffer);
  if (rval == TH_EFAULT)
    ROS_WARN("EFault in submitting uncompressed frame to encoder.");
  if (rval == TH_EINVAL)
    ROS_WARN("EInval in submitting uncompressed frame to encoder.");

  ogg_packet oggpacket;
  theora_image_transport::Packet output;
  ROS_DEBUG("Ready to get encoded packets.");
  while ((rval = th_encode_packetout(encoding_context_.get(), 0, &oggpacket)) > 0) {
    oggPacketToMsg(message.header, oggpacket, output);
    publish_fn(output);
  }
  if (rval == TH_EFAULT)
    ROS_WARN("EFAULT in retreiving encoded video data packets.");
}

void freeContext(th_enc_ctx* context)
{
  if (context) th_encode_free(context);
}

void TheoraPublisher::ensure_encoding_context(const sensor_msgs::Image& image, const PublishFn& publish_fn) const
{
  /// @todo Check if image size or encoding has changed
  if (encoding_context_) return;

  // Theora has a divisible-by-sixteen restriction for the encoded frame size, so
  // scale the picture size up to the nearest multiple of 16 and calculate offsets.
  encoder_setup_.frame_width = (image.width + 15) & ~0xF;
  encoder_setup_.frame_height = (image.height + 15) & ~0xF;
  encoder_setup_.pic_width = image.width;
  encoder_setup_.pic_height = image.height;

  // Allocate encoding context. Smart pointer ensures that th_encode_free gets called.
  encoding_context_.reset(th_encode_alloc(&encoder_setup_), freeContext);

  /// @todo More of a problem than a ROS_DEBUG!
  if (!encoding_context_)
    ROS_WARN("Encoding context not successfully created.");

  th_comment comment;
  th_comment_init(&comment);
  boost::shared_ptr<th_comment> clear_guard(&comment, th_comment_clear);
  /// @todo Store image encoding in comment
  //th_comment_add(&comment, (char*)"Compression node written by Ethan.");
  comment.vendor = strdup("Willow Garage theora_image_transport");

  // Construct the header and stream it in case anyone is already listening
  /// @todo Try not to send headers twice to some listeners
  stream_header_.clear();
  ogg_packet oggpacket;
  while (th_encode_flushheader(encoding_context_.get(), &comment, &oggpacket) > 0) {
    stream_header_.push_back(theora_image_transport::Packet());
    oggPacketToMsg(image.header, oggpacket, stream_header_.back());
    publish_fn(stream_header_.back());
  }
  ROS_INFO("In ensure_encoding_context, published %d header packets", (int)stream_header_.size());
}

void TheoraPublisher::oggPacketToMsg(const roslib::Header& header, const ogg_packet &oggpacket,
                                     theora_image_transport::Packet &msg) const
{
  msg.header     = header;
  msg.b_o_s      = oggpacket.b_o_s;
  msg.e_o_s      = oggpacket.e_o_s;
  msg.granulepos = oggpacket.granulepos;
  msg.packetno   = oggpacket.packetno;
  msg.data.resize(oggpacket.bytes);
  memcpy(&msg.data[0], oggpacket.packet, oggpacket.bytes);
}

} //namespace theora_image_transport
