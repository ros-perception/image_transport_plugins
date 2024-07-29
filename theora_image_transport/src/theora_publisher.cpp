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

#include "theora_image_transport/theora_publisher.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "theora_image_transport/compression_common.hpp"

namespace theora_image_transport
{

enum theoraParameters
{
  OPTIMIZE_FOR = 0,
  TARGET_BITRATE,
  QUALITY,
  KEYFRAME_FREQUENCY
};

enum optimizeForTarget : bool
{
  OPTIMIZE_BITRATE = 0,
  OPTIMIZE_QUALITY
};

const struct ParameterDefinition kParameters[] =
{
  {  // OPTIMIZE_FOR - Try to achieve either 'target_bitrate' (0) or 'quality' (1)
    ParameterValue(true),  // Default to quality
    ParameterDescriptor()
    .set__name("optimize_for")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    .set__description("Try to achieve either 'target_bitrate' (0) or 'quality' (1)")
    .set__read_only(false)
  },
  {  // TARGET_BITRATE - Target encoding bitrate, bits per second
    ParameterValue(static_cast<int>(800000)),
    ParameterDescriptor()
    .set__name("target_bitrate")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Target encoding bitrate, bits per second")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(99200000)
        .set__step(1)})
  },
  {  // QUALITY - Encoding quality
    ParameterValue(static_cast<int>(31)),
    ParameterDescriptor()
    .set__name("quality")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Encoding quality")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(63)
        .set__step(1)})
  },
  {  // KEYFRAME_FREQUENCY - Maximum distance between key frames
    ParameterValue(static_cast<int>(64)),
    ParameterDescriptor()
    .set__name("keyframe_frequency")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Maximum distance between key frames")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(1)
        .set__to_value(64)
        .set__step(1)})
  },
};

TheoraPublisher::TheoraPublisher()
:logger_(rclcpp::get_logger("TheoraPublisher"))
{
  refreshConfigNeeded = false;

  // Initialize info structure fields that don't change
  th_info_init(&encoder_setup_);

  encoder_setup_.pic_x = 0;
  encoder_setup_.pic_y = 0;
  encoder_setup_.colorspace = TH_CS_UNSPECIFIED;
  // See bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
  encoder_setup_.pixel_fmt = TH_PF_420;
  encoder_setup_.aspect_numerator = 1;
  encoder_setup_.aspect_denominator = 1;
  encoder_setup_.fps_numerator = 1;  // don't know the frame rate ahead of time
  encoder_setup_.fps_denominator = 1;
  encoder_setup_.keyframe_granule_shift = 6;  // A good default for streaming applications
  // Note: target_bitrate and quality set to correct values in refreshConfig
  encoder_setup_.target_bitrate = -1;
  encoder_setup_.quality = -1;
}

TheoraPublisher::~TheoraPublisher()
{
  th_info_clear(&encoder_setup_);
}

void TheoraPublisher::advertiseImpl(
  rclcpp::Node *node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;
  logger_ = node->get_logger();

  typedef image_transport::SimplePublisherPlugin<theora_image_transport::msg::Packet> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  using callbackT = std::function<void(ParameterEvent::SharedPtr event)>;
  auto callback = std::bind(&TheoraPublisher::onParameterEvent, this, std::placeholders::_1,
                            node->get_fully_qualified_name(), param_base_name);

  parameter_subscription_ = rclcpp::SyncParametersClient::on_parameter_event<callbackT>(node,
      callback);

  for(const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }

  // we need to trigger initial configuration loading like ROS1 setCallback does
  refreshConfigNeeded = true;
  refreshConfig();
}

static void cvToTheoraPlane(cv::Mat & mat, th_img_plane & plane)
{
  plane.width = mat.cols;
  plane.height = mat.rows;
  plane.stride = mat.step;
  plane.data = mat.data;
}

void TheoraPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  if (!ensureEncodingContext(message, publish_fn)) {
    return;
  }
  // return;
  /// @todo fromImage is deprecated
  /// @todo Optimized gray-scale path, rgb8
  /// @todo fromImage can throw cv::Exception on bayer encoded images

  refreshConfig();

  cv_bridge::CvImageConstPtr cv_image_ptr;
  try {
    // conversion necessary
    cv_image_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: '%s'", e.what());
    return;
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(logger_, "OpenCV exception: '%s'", e.what());
    return;
  }

  if (cv_image_ptr == 0) {
    RCLCPP_ERROR(logger_, "Unable to convert from '%s' to 'bgr8'", message.encoding.c_str());
    return;
  }

  const cv::Mat bgr = cv_image_ptr->image;

  cv::Mat bgr_padded;
  int frame_width = encoder_setup_.frame_width, frame_height = encoder_setup_.frame_height;
  if (frame_width == bgr.cols && frame_height == bgr.rows) {
    bgr_padded = bgr;
  } else {
    bgr_padded = cv::Mat::zeros(frame_height, frame_width, bgr.type());
    cv::Mat pic_roi = bgr_padded(cv::Rect(0, 0, bgr.cols, bgr.rows));
    bgr.copyTo(pic_roi);
  }

  // Convert image to Y'CbCr color space used by Theora
  cv::Mat ycrcb;
  cv::cvtColor(bgr_padded, ycrcb, cv::COLOR_BGR2YCrCb);

  // Split channels
  cv::Mat ycrcb_planes[3];
  cv::split(ycrcb, ycrcb_planes);

  // Use Y as-is but subsample chroma channels
  cv::Mat y = ycrcb_planes[0], cr, cb;
  cv::pyrDown(ycrcb_planes[1], cr);
  cv::pyrDown(ycrcb_planes[2], cb);

  // Construct Theora image buffer
  th_ycbcr_buffer ycbcr_buffer;
  cvToTheoraPlane(y, ycbcr_buffer[0]);
  cvToTheoraPlane(cb, ycbcr_buffer[1]);
  cvToTheoraPlane(cr, ycbcr_buffer[2]);

  // Submit frame to the encoder
  int rval = th_encode_ycbcr_in(encoding_context_.get(), ycbcr_buffer);
  if (rval == TH_EFAULT) {
    RCLCPP_ERROR(logger_, "[theora] EFAULT in submitting uncompressed frame to encoder");
    return;
  }
  if (rval == TH_EINVAL) {
    RCLCPP_ERROR(logger_, "[theora] EINVAL in submitting uncompressed frame to encoder");
    return;
  }

  // Retrieve and publish encoded video data packets
  ogg_packet oggpacket;
  theora_image_transport::msg::Packet output;
  while ((rval = th_encode_packetout(encoding_context_.get(), 0, &oggpacket)) > 0) {
    oggPacketToMsg(message.header, oggpacket, output);
    publish_fn(output);
  }
  if (rval == TH_EFAULT) {
    RCLCPP_ERROR(logger_, "[theora] EFAULT in retrieving encoded video data packets");
  }
}

void TheoraPublisher::refreshConfig() const
{
  // we need guard to prevent !TH_ENCCTL_SET_BITRATE path resetting context every time
  // the flag is set on init and on config change event, refresh happens on publish
  if(!refreshConfigNeeded) {
    return;
  }
  refreshConfigNeeded = false;

  // Fresh Configuration
  optimizeForTarget cfg_optimize_for =
    (optimizeForTarget)node_->get_parameter(parameters_[OPTIMIZE_FOR]).get_value<bool>();
  int cfg_bitrate = node_->get_parameter(parameters_[TARGET_BITRATE]).get_value<int>();
  int cfg_quality = node_->get_parameter(parameters_[QUALITY]).get_value<int>();
  int cfg_keyframe_frequency =
    node_->get_parameter(parameters_[KEYFRAME_FREQUENCY]).get_value<int>();

  long bitrate = 0;  // NOLINT
  if (cfg_optimize_for == OPTIMIZE_BITRATE) {
    bitrate = cfg_bitrate;
  }
  bool update_bitrate = bitrate && encoder_setup_.target_bitrate != bitrate;
  bool update_quality = !bitrate &&
    ((encoder_setup_.quality != cfg_quality) || encoder_setup_.target_bitrate > 0);
  // store configured quality and bitrate ONLY if optimizing for it, otherwise zero-out bitrate
  encoder_setup_.quality = cfg_quality;
  encoder_setup_.target_bitrate = bitrate;
  keyframe_frequency_ = cfg_keyframe_frequency;

  if (encoding_context_) {
    int err = 0;
    // libtheora 1.1 lets us change quality or bitrate on the fly, 1.0 does not.
#ifdef TH_ENCCTL_SET_BITRATE
    if (update_bitrate) {
      err = th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_BITRATE, &bitrate, sizeof(long));  // NOLINT
      if (err) {
        RCLCPP_ERROR(logger_, "Failed to update bitrate dynamically");
      }
    }
#else
    err |= update_bitrate;
#endif

#ifdef TH_ENCCTL_SET_QUALITY
    if (update_quality) {
      err = th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_QUALITY, &cfg_quality,
          sizeof(int));
      // In 1.1 above call will fail if a bitrate has previously been set. That restriction may
      // be relaxed in a future version. Complain on other failures.
      if (err && err != TH_EINVAL) {
        RCLCPP_ERROR(logger_, "Failed to update quality dynamically");
      }
    }
#else
    err |= update_quality;
#endif

    // If unable to change parameters dynamically, just create a new encoding context.
    if (err) {
      encoding_context_.reset();
    } else {  // Otherwise, do the easy updates and keep going!
      updateKeyframeFrequency();
      // In case desired value was unattainable
      if(cfg_keyframe_frequency != static_cast<int>(keyframe_frequency_)) {
        node_->set_parameter(rclcpp::Parameter(parameters_[KEYFRAME_FREQUENCY],
          static_cast<int>(keyframe_frequency_)));
      }
    }
  }
}

void freeContext(th_enc_ctx * context)
{
  if (context) {th_encode_free(context);}
}

bool TheoraPublisher::ensureEncodingContext(
  const sensor_msgs::msg::Image & image,
  const PublishFn & publish_fn) const
{
  /// @todo Check if encoding has changed
  if (encoding_context_ && encoder_setup_.pic_width == image.width &&
    encoder_setup_.pic_height == image.height)
  {
    return true;
  }

  // Theora has a divisible-by-sixteen restriction for the encoded frame size, so
  // scale the picture size up to the nearest multiple of 16 and calculate offsets.
  encoder_setup_.frame_width = (image.width + 15) & ~0xF;
  encoder_setup_.frame_height = (image.height + 15) & ~0xF;
  encoder_setup_.pic_width = image.width;
  encoder_setup_.pic_height = image.height;

  // Allocate encoding context. Smart pointer ensures that th_encode_free gets called.
  encoding_context_.reset(th_encode_alloc(&encoder_setup_), freeContext);
  if (!encoding_context_) {
    RCLCPP_ERROR(logger_, "[theora] Failed to create encoding context");
    return false;
  }

  updateKeyframeFrequency();

  th_comment comment;
  th_comment_init(&comment);
  std::shared_ptr<th_comment> clear_guard(&comment, th_comment_clear);
  /// @todo Store image encoding in comment
  comment.vendor = strdup("Willow Garage theora_image_transport");

  // Construct the header and stream it in case anyone is already listening
  /// @todo Try not to send headers twice to some listeners
  stream_header_.clear();
  ogg_packet oggpacket;
  while (th_encode_flushheader(encoding_context_.get(), &comment, &oggpacket) > 0) {
    stream_header_.push_back(theora_image_transport::msg::Packet());
    oggPacketToMsg(image.header, oggpacket, stream_header_.back());
    publish_fn(stream_header_.back());
  }
  return true;
}

void TheoraPublisher::oggPacketToMsg(
  const std_msgs::msg::Header & header,
  const ogg_packet & oggpacket,
  theora_image_transport::msg::Packet & msg) const
{
  msg.header = header;
  msg.b_o_s = oggpacket.b_o_s;
  msg.e_o_s = oggpacket.e_o_s;
  msg.granulepos = oggpacket.granulepos;
  msg.packetno = oggpacket.packetno;
  msg.data.resize(oggpacket.bytes);
  memcpy(&msg.data[0], oggpacket.packet, oggpacket.bytes);
}

void TheoraPublisher::updateKeyframeFrequency() const
{
  ogg_uint32_t desired_frequency = keyframe_frequency_;
  if (th_encode_ctl(encoding_context_.get(), TH_ENCCTL_SET_KEYFRAME_FREQUENCY_FORCE,
                    &keyframe_frequency_, sizeof(ogg_uint32_t)))
  {
    RCLCPP_ERROR(logger_, "Failed to change keyframe frequency");
  }
  if (keyframe_frequency_ != desired_frequency) {
    RCLCPP_WARN(logger_, "Couldn't set keyframe frequency to %d, actually set to %d",
                      desired_frequency, keyframe_frequency_);
  }
}

void TheoraPublisher::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.theora.quality)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  // deprecated non-scoped parameter name (e.g. image_raw.quality)
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

void TheoraPublisher::onParameterEvent(
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
    // e.g. `color.image_raw.` + `theora` + `quality`
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

  // if any of the non-deprecated parameters changed mark to refresh config
  rclcpp::ParameterEventsFilter filterChanged(event, parameters_, {EventType::CHANGED});
  refreshConfigNeeded = filterChanged.get_events().size() > 0;
}

}  // namespace theora_image_transport
