// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "theora_image_transport/theora_subscriber.hpp"

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "theora_image_transport/compression_common.hpp"

namespace theora_image_transport
{

enum theoraParameters
{
  POST_PROCESSING_LEVEL = 0,
};

const struct ParameterDefinition kParameters[] =
{
  {
  // POST_PROCESSING_LEVEL - Post-processing level.
  // Higher values can improve the appearance of the decoded images at the cost of more CPU.
    ParameterValue(static_cast<int>(0)),
    ParameterDescriptor()
    .set__name("post_processing_level")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description(
        "Post-processing level. Higher values can improve the appearance of the decoded "
        "images at the cost of more CPU.")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(7)
        .set__step(1)})
  }
};

TheoraSubscriber::TheoraSubscriber()
: pplevel_(0),
  received_header_(false),
  received_keyframe_(false),
  decoding_context_(NULL),
  setup_info_(NULL),
  logger_(rclcpp::get_logger("TheoraSubscriber"))
{
  th_info_init(&header_info_);
  th_comment_init(&header_comment_);
}

TheoraSubscriber::~TheoraSubscriber()
{
  if (decoding_context_) {th_decode_free(decoding_context_);}
  th_setup_free(setup_info_);
  th_info_clear(&header_info_);
  th_comment_clear(&header_comment_);
}

void TheoraSubscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  node_ = node;
  logger_ = node->get_logger();

  typedef image_transport::SimpleSubscriberPlugin<theora_image_transport::msg::Packet> Base;
  Base::subscribeImpl(node, base_topic, callback, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  using paramCallbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto paramCallback = std::bind(&TheoraSubscriber::onParameterEvent, this, std::placeholders::_1,
                                 node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<paramCallbackT>(node,
      paramCallback);

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

void TheoraSubscriber::refreshConfig()
{
  int cfg_pplevel = node_->get_parameter(parameters_[POST_PROCESSING_LEVEL]).get_value<int>();

  if (decoding_context_ && pplevel_ != cfg_pplevel) {
    pplevel_ = updatePostProcessingLevel(cfg_pplevel);
    // In case more than PPLEVEL_MAX
    node_->set_parameter(rclcpp::Parameter(parameters_[POST_PROCESSING_LEVEL], pplevel_));
  } else {
    pplevel_ = cfg_pplevel;
  }
}

int TheoraSubscriber::updatePostProcessingLevel(int level)
{
  int pplevel_max;
  int err = th_decode_ctl(decoding_context_, TH_DECCTL_GET_PPLEVEL_MAX, &pplevel_max, sizeof(int));
  if (err) {
    RCLCPP_WARN(logger_, "Failed to get maximum post-processing level, error code %d", err);
  } else if (level > pplevel_max) {
    RCLCPP_WARN(logger_, "Post-processing level %d is above the maximum, clamping to %d", level,
        pplevel_max);
    level = pplevel_max;
  }

  err = th_decode_ctl(decoding_context_, TH_DECCTL_SET_PPLEVEL, &level, sizeof(int));
  if (err) {
    RCLCPP_ERROR(logger_, "Failed to set post-processing level, error code %d", err);
    return pplevel_;  // old value
  }
  return level;
}

// When using this caller is responsible for deleting oggpacket.packet!!
void TheoraSubscriber::msgToOggPacket(
  const theora_image_transport::msg::Packet & msg,
  ogg_packet & ogg)
{
  ogg.bytes = msg.data.size();
  ogg.b_o_s = msg.b_o_s;
  ogg.e_o_s = msg.e_o_s;
  ogg.granulepos = msg.granulepos;
  ogg.packetno = msg.packetno;
  ogg.packet = new unsigned char[ogg.bytes];
  memcpy(ogg.packet, &msg.data[0], ogg.bytes);
}

void TheoraSubscriber::internalCallback(
  const theora_image_transport::msg::Packet::ConstSharedPtr & message,
  const Callback & callback)
{
  refreshConfig();

  /// @todo Break this function into pieces
  ogg_packet oggpacket;
  msgToOggPacket(*message, oggpacket);
  // Make sure packet memory gets deleted
  std::unique_ptr<unsigned char[]> packet_guard(oggpacket.packet);

  // Beginning of logical stream flag means we're getting new headers
  if (oggpacket.b_o_s == 1) {
    // Clear all state, everything we knew is wrong
    received_header_ = false;
    received_keyframe_ = false;
    if (decoding_context_) {
      th_decode_free(decoding_context_);
      decoding_context_ = NULL;
    }
    th_setup_free(setup_info_);
    setup_info_ = NULL;
    th_info_clear(&header_info_);
    th_info_init(&header_info_);
    th_comment_clear(&header_comment_);
    th_comment_init(&header_comment_);
    latest_image_.reset();
  }

  // Decode header packets until we get the first video packet
  if (received_header_ == false) {
    int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    switch (rval) {
      case 0:
        // We've received the full header; this is the first video packet.
        decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
        if (!decoding_context_) {
          RCLCPP_ERROR(logger_, "[theora] Decoding parameters were invalid");
          return;
        }
        received_header_ = true;
        pplevel_ = updatePostProcessingLevel(pplevel_);
        break;  // Continue on the video decoding
      case TH_EFAULT:
        RCLCPP_WARN(logger_, "[theora] EFAULT when processing header packet");
        return;
      case TH_EBADHEADER:
        RCLCPP_WARN(logger_, "[theora] Bad header packet");
        return;
      case TH_EVERSION:
        RCLCPP_WARN(logger_, "[theora] Header packet not decodable with this version "
          "of libtheora");
        return;
      case TH_ENOTFORMAT:
        RCLCPP_WARN(logger_, "[theora] Packet was not a Theora header");
        return;
      default:
        // If rval > 0, we successfully received a header packet.
        if (rval < 0) {
          RCLCPP_WARN(logger_, "[theora] Error code %d when processing header packet",
            rval);
        }
        return;
    }
  }

  // Wait for a keyframe if we haven't received one yet - delta frames are useless to
  // us in that case
  received_keyframe_ = received_keyframe_ || (th_packet_iskeyframe(&oggpacket) == 1);
  if (!received_keyframe_) {
    return;
  }

  // We have a video packet we can handle, let's decode it
  int rval = th_decode_packetin(decoding_context_, &oggpacket, NULL);
  switch (rval) {
    case 0:
      break;  // Yay, we got a frame. Carry on below.
    case TH_DUPFRAME:
      // Video data hasn't changed, so we update the timestamp and reuse the last received frame.
      RCLCPP_DEBUG(logger_, "[theora] Got a duplicate frame");
      if (latest_image_) {
        latest_image_->header = message->header;
        callback(latest_image_);
      }
      return;
    case TH_EFAULT:
      RCLCPP_WARN(logger_, "[theora] EFAULT processing video packet");
      return;
    case TH_EBADPACKET:
      RCLCPP_WARN(logger_, "[theora] Packet does not contain encoded video data");
      return;
    case TH_EIMPL:
      RCLCPP_WARN(logger_,
        "[theora] The video data uses bitstream features not supported by this version of "
        "libtheora");
      return;
    default:
      RCLCPP_WARN(logger_, "[theora] Error code %d when decoding video packet", rval);
      return;
  }

  // We have a new decoded frame available
  th_ycbcr_buffer ycbcr_buffer;
  th_decode_ycbcr_out(decoding_context_, ycbcr_buffer);

  // Wrap YCbCr channel data into OpenCV format
  th_img_plane & y_plane = ycbcr_buffer[0], & cb_plane = ycbcr_buffer[1],
  & cr_plane = ycbcr_buffer[2];
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

  latest_image_ =
    cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::BGR8, bgr).toImageMsg();
  /// @todo Handle RGB8 or MONO8 efficiently
  callback(latest_image_);
}

void TheoraSubscriber::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.theora.post_processing_level)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  // deprecated non-scoped parameter name (e.g. image_raw.post_processing_level)
  const std::string deprecated_name = base_name + "." + definition.descriptor.name;
  deprecatedParameters_.push_back(deprecated_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(param_name, definition.defaultValue,
        definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }

  // transport scoped parameter as default, otherwise we would overwrite
  try {
    node_->declare_parameter(deprecated_name, param_value, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    node_->get_parameter(deprecated_name).get_parameter_value();
  }
}

void TheoraSubscriber::onParameterEvent(
  ParameterEvent::SharedPtr event, std::string full_name,
  std::string base_name)
{
  // filter out events from other nodes
  if (event->node != full_name) {
    return;
  }

  // filter out new/changed deprecated parameters
  using EventType = rclcpp::ParameterEventsFilter::EventType;

  rclcpp::ParameterEventsFilter filter(event, deprecatedParameters_,
    {EventType::NEW, EventType::CHANGED});

  const std::string transport = getTransportName();

  // emit warnings for deprecated parameters & sync deprecated parameter value to correct
  for (auto & it : filter.get_events()) {
    const std::string name = it.second->name;

    // name was generated from base_name, has to succeed
    size_t baseNameIndex = name.find(base_name);
    size_t paramNameIndex = baseNameIndex + base_name.size();
    // e.g. `color.image_raw.` + `theora` + `post_processing_level`
    std::string recommendedName = name.substr(0,
        paramNameIndex + 1) + transport + name.substr(paramNameIndex);

    rclcpp::Parameter recommendedValue = node_->get_parameter(recommendedName);

    // do not emit warnings if deprecated value matches
    if(it.second->value == recommendedValue.get_value_message()) {
      continue;
    }

    RCLCPP_WARN_STREAM(logger_, "parameter `" << name << "` is deprecated" <<
                                "; use transport qualified name `" << recommendedName << "`");

    node_->set_parameter(rclcpp::Parameter(recommendedName, it.second->value));
  }
}

}  // namespace theora_image_transport
