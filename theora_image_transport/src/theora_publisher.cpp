#include "theora_image_transport/theora_publisher.h"
#include <sensor_msgs/image_encodings.h>

#include <vector>
#include <cstdio> //for memcpy

using namespace std;

namespace theora_image_transport {

/// @todo publishHeader function so we can unify with code in ensure_encoding_context
/// @todo the sleeping is excessive, do we really need it? how big is the buffer? ros::Publication::processPublishQueue()
// Sends the header packets (if any) to new subscribers
void TheoraPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  ROS_INFO("In connectCallback");
  ros::Duration d(0.1);
  for (unsigned int i = 0; i < stream_header_.size(); i++) {
    pub.publish(stream_header_[i]);
    ROS_INFO("Publishing header packet");
    d.sleep(); // sleep briefly after each packet 
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
  /// @todo Optimized gray-scale path
  if (!img_bridge_.fromImage(message, "bgr8")) {
    ROS_ERROR("Unable to convert from '%s' to bgr8", message.encoding.c_str());
    return;
  }

  cv::Mat bgr(img_bridge_.toIpl()), bgr_padded;
  ensure_encoding_context(bgr.size(), publish_fn);
  // Pad dimensions out to multiples of 16
  if (nearest_width_ == bgr.cols && nearest_height_ == bgr.rows) {
    bgr_padded = bgr;
  }
  else {
    bgr_padded = cv::Mat::zeros(nearest_height_, nearest_width_, bgr.type());
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
  
  ROS_DEBUG("Width: %d, Height: %d", nearest_width_, nearest_height_);
#if 0
  ROS_INFO("Buffer planes:");
  for (int i = 0; i < 3; ++i)
    printf("\t%d: width = %d, height = %d, stride = %d\n", i, ycbcr_buffer[i].width, ycbcr_buffer[i].height,
           ycbcr_buffer[i].stride);
#endif
  //ROS_INFO("Publishing image");

  /// @todo Louder errors?
  int rval;
  if (!encoding_context_)
    ROS_DEBUG("About to encode with null encoding context.");
  rval = th_encode_ycbcr_in(encoding_context_.get(), ycbcr_buffer);
  if (rval == TH_EFAULT)
    ROS_WARN("EFault in encoding.");
  if (rval == TH_EINVAL)
    ROS_WARN("EInval in encoding.");
  ROS_DEBUG("Encoding resulted in: %d", rval);

  ogg_packet oggpacket;
  theora_image_transport::packet output;
  ROS_DEBUG("Ready to get encoded packets.");
  while ((rval = th_encode_packetout(encoding_context_.get(), 0, &oggpacket)) > 0) {
    oggPacketToMsg(oggpacket, output);
    publish_fn(output);
  }
  ROS_DEBUG("Punted from while loop with rval %d", rval);
}

void free_context(th_enc_ctx* context)
{
  if (context) th_encode_free(context);
}

void TheoraPublisher::ensure_encoding_context(const CvSize &size, const PublishFn& publish_fn) const
{
  if (encoding_context_) return;

  th_info encoder_setup;
  th_info_init(&encoder_setup);

  // Theora has a divisible-by-sixteen restriction for the encoded frame size, so
  // scale the picture size up to the nearest multiple of 16 and calculate offsets.
  nearest_width_ = (size.width + 15) & ~0xF;
  nearest_height_ = (size.height + 15) & ~0xF;

  /// @todo Make encoder_setup a class member, move most of this to initialization
  encoder_setup.frame_width = nearest_width_;
  encoder_setup.frame_height = nearest_height_;
  encoder_setup.pic_width = size.width;
  encoder_setup.pic_height = size.height;
  encoder_setup.pic_x = 0;
  encoder_setup.pic_y = 0;
  ROS_DEBUG("Creating context with Width: %d, Height: %d", nearest_width_, nearest_height_);
  encoder_setup.colorspace = TH_CS_UNSPECIFIED;
  encoder_setup.pixel_fmt = TH_PF_420; //see bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
  /// @todo Make target bitrate and quality parameters
  int bitrate;
  nh().param("theora_bitrate", bitrate, 800000);
  encoder_setup.target_bitrate = bitrate;
  //encoder_setup.quality = 63;    //On a scale of 0 to 63, to use this set target bitrate to 0
  encoder_setup.aspect_numerator = 1;
  encoder_setup.aspect_denominator = 1;
  encoder_setup.fps_numerator = 0;
  encoder_setup.fps_denominator = 0;
  encoder_setup.keyframe_granule_shift = 6; //Apparently a good default

  encoding_context_.reset(th_encode_alloc(&encoder_setup), free_context);

  /// @todo Is this more of a problem than a ROS_DEBUG?
  if (!encoding_context_)
    ROS_DEBUG("Encoding context not successfully created.");

  /// @todo Stick header info (time stamp, frame id) in comment, also encoding?
  /// @todo Also put connection header in comment
  th_comment comment;
  th_comment_init(&comment);
  th_comment_add(&comment, (char*)"Compression node written by Ethan.");
  comment.vendor = (char*)"Encoded by Willow Garage image_compression_node.";

  // Construct the header and stream it in case anyone is already listening
  stream_header_.clear();
  ogg_packet oggpacket;
  ros::Duration d(0.1);
  ROS_INFO("In ensure_encoding_context");
  while (th_encode_flushheader(encoding_context_.get(), &comment, &oggpacket) > 0) {
    stream_header_.push_back(theora_image_transport::packet());
    oggPacketToMsg(oggpacket, stream_header_.back());
    publish_fn(stream_header_.back());
    ROS_INFO("Publishing header packet");
    d.sleep(); // sleep for 0.1s after each packet
  }
  //ROS_DEBUG("Published %d header packets.", stream_header_.size());
  //th_comment_clear(&comment);  /// @todo this should happen but is causing crazy seg faults (probably trying to free a string literal)

  if (!encoding_context_)
    ROS_DEBUG("Encoding context killed by header flushing.");
}

void TheoraPublisher::oggPacketToMsg(const ogg_packet &oggpacket, theora_image_transport::packet &msgOutput) const
{
  msgOutput.blob.resize(oggpacket.bytes);
  memcpy(&msgOutput.blob[0], oggpacket.packet, oggpacket.bytes);
  msgOutput.bytes = oggpacket.bytes;
  msgOutput.b_o_s = oggpacket.b_o_s;
  msgOutput.e_o_s = oggpacket.e_o_s;
  msgOutput.granulepos = oggpacket.granulepos;
  msgOutput.packetno = oggpacket.packetno;
}

} //namespace theora_image_transport
