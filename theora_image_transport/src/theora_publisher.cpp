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

void TheoraPublisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  /// @todo Use right function, not fromImage
  /// @todo Switch code to use cv::Mat, should be much simpler
  /// @todo Optimized gray-scale path
  if (!img_bridge_.fromImage(message, "bgr8")) {
    ROS_ERROR("Unable to convert from %s to bgr", message.encoding.c_str());
    return;
  }
  
  IplImage* img = img_bridge_.toIpl();
  IplImage* img2 = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
  cvCvtColor(img, img2, CV_BGR2YCrCb);
  /// @todo Not checking if image size changes
  ensure_encoding_context(cvGetSize(img2), publish_fn);

  //convert image
  th_ycbcr_buffer ycbcr_image;
  vector<unsigned char> planes[3]; //color planes
  //Size is number of pixels in padded image (chroma subsampled by factor of 2 in each dimension, which means area is 1/4)
  for (int i = 0; i < 3; i++)
    planes[i].resize(nearest_width_ * nearest_height_ / (i > 0 ? 4 : 1));

  //ROS_DEBUG("Image size: %d image width: %d image height: %d", img2->imageSize, img2->width, img2->height);
  for (int i = 0; i < img2->imageSize / 3; i++) //imageSize/3 is the number of pixels
  {
    planes[0][i] = *(unsigned char*)(img2->imageData + i * 3); //Y
    //planes[2][i] = *(unsigned char*)(img2->imageData + i*3 + 1);  //Cr
    //planes[1][i] = *(unsigned char*)(img2->imageData + i*3 + 2);  //Cb
  }

  //Note that while OpenCV uses YCbCr, theora uses YCrCb... this can make things a little confusing
  //CHROMA subsampling
  for (int planeIdx = 1; planeIdx < 3; planeIdx++)
  {
    int swappedIdx = planeIdx;
    if (planeIdx == 1)
      swappedIdx = 2;
    else if (planeIdx == 2)
      swappedIdx = 1;
    for (int i = 0; i < img2->width; i += 2)
      for (int j = 0; j < img2->height; j += 2)
      {
        int planeDataIdx = i / 2 + (j * img2->width) / 4;
        //planes[2][planeDataIdx]
        unsigned int total = (unsigned int)((uchar*)(img2->imageData + img2->widthStep * j))[i * 3 + planeIdx];
        unsigned int count = 1;
        if (i < img2->width - 1)
        {
          total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * j))[(i + 1) * 3 + planeIdx];
          count++;
        }
        if (j < img2->height - 1)
        {
          total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * (j + 1)))[i * 3 + planeIdx];
          count++;
        }
        if (i < img2->width - 1 && j < img2->height - 1)
        {
          total += (unsigned int)((uchar*)(img2->imageData + img2->widthStep * (j + 1)))[(i + 1) * 3 + planeIdx];
          count++;
        }
        planes[swappedIdx][planeDataIdx] = (uchar)(total / count);
      }
  }

  for (int i = 0; i < 3; i++)
  {
    ycbcr_image[i].width = nearest_width_ / (i > 0 ? 2 : 1); //
    ycbcr_image[i].height = nearest_height_ / (i > 0 ? 2 : 1); //chroma is subsampled by a factor of 2
    ycbcr_image[i].stride = img2->width / (i > 0 ? 2 : 1); //
    ycbcr_image[i].data = &planes[i][0];
  }
  ROS_DEBUG("Width: %d, Height: %d, xOff: %d, yOff: %d", nearest_width_, nearest_height_,
            nearest_x_offset_, nearest_y_offset_);

  cvReleaseImage(&img2);

  /// @todo Louder errors?
  int rval;
  if (!encoding_context_)
    ROS_DEBUG("About to encode with null encoding context.");
  rval = th_encode_ycbcr_in(encoding_context_.get(), ycbcr_image);
  if (rval == TH_EFAULT)
    ROS_DEBUG("EFault in encoding.");
  if (rval == TH_EINVAL)
    ROS_DEBUG("EInval in encoding.");
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
  nearest_width_ = size.width + size.width % 16 == 0 ? 0 : (16 - size.width % 16);
  nearest_height_ = size.height + size.height % 16 == 0 ? 0 : (16 - size.height % 16);

  // Theora has a divisible-by-sixteen restriction for the encoded frame size, so
  // scale the picture size up to the nearest multiple of 16 and calculate offsets.
  nearest_width_ = (size.width + 15) & ~0xF;
  nearest_height_ = (size.height + 15) & ~0xF;
  // Force the offsets to be even so that chroma samples line up like we expect.
  nearest_x_offset_ = ((nearest_width_ - size.width) / 2) & ~1;
  nearest_y_offset_ = ((nearest_height_ - size.height) / 2) & ~1;

  encoder_setup.frame_width = nearest_width_;
  encoder_setup.frame_height = nearest_height_;
  encoder_setup.pic_width = size.width;
  encoder_setup.pic_height = size.height;
  encoder_setup.pic_x = nearest_x_offset_;
  encoder_setup.pic_y = nearest_y_offset_;
  ROS_DEBUG("Creating context with Width: %d, Height: %d", nearest_width_, nearest_height_);
  encoder_setup.colorspace = TH_CS_UNSPECIFIED;
  //encoder_setup.colorspace = TH_CS_ITU_REC_470M;     //TH_CS_ITU_REC_470M     A color space designed for NTSC content.
  //TH_CS_ITU_REC_470BG     A color space designed for PAL/SECAM content.
  encoder_setup.pixel_fmt = TH_PF_420; //see bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
  int bitrate;
  nh().param("theora_bitrate", bitrate, 800000); /// @todo Caching version, search upwards?
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

  /// @todo Stick header info in comment
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
  //ROS_DEBUG("Ready to send %d bytes in packet#%d and granule%d (and this is BOS: %d).", msgOutput.bytes, msgOutput.packetno, msgOutput.granulepos, msgOutput.b_o_s);
  /*unsigned int i = 0;
  for (int j = 0; j < msgOutput.bytes; j++)
    i = i * 2 % 91 + msgOutput.blob[j];
  ROS_DEBUG("Checksum is: %d", i);*/
}

} //namespace theora_image_transport
