// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "svtav1_image_transport/svtav1_publisher.hpp"

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "cv_bridge/cv_bridge.hpp"

namespace svtav1_image_transport
{

// enum hParameters
// {
// };

// const struct ParameterDefinition kParameters[] =
// {
// };

#define PROP_RC_MODE_CQP 0
#define PROP_RC_MODE_VBR 1

#define PROP_ENCMODE_DEFAULT                11
#define PROP_HIERARCHICAL_LEVEL_DEFAULT     4
#define PROP_P_FRAMES_DEFAULT               0
#define PROP_PRED_STRUCTURE_DEFAULT         2
#define PROP_GOP_SIZE_DEFAULT               -1
#define PROP_INTRA_REFRESH_DEFAULT          2
#define PROP_QP_DEFAULT                     50
#define PROP_DEBLOCKING_DEFAULT             1
#define PROP_RC_MODE_DEFAULT                PROP_RC_MODE_CQP
#define PROP_BITRATE_DEFAULT                700000
#define PROP_QP_MAX_DEFAULT                 63
#define PROP_QP_MIN_DEFAULT                 0
#define PROP_LOOKAHEAD_DEFAULT              (unsigned int)-1
#define PROP_SCD_DEFAULT                    0
#define PROP_AUD_DEFAULT                    0
#define PROP_CORES_DEFAULT                  10
#define PROP_SOCKET_DEFAULT                 -1

void
set_default_svt_configuration(EbSvtAv1EncConfiguration * svt_config, int width, int height)
{
  memset(svt_config, 0, sizeof(EbSvtAv1EncConfiguration));
  svt_config->source_width = width;
  svt_config->source_height = height;
  svt_config->intra_period_length = PROP_GOP_SIZE_DEFAULT - 1;
  svt_config->intra_refresh_type = static_cast<SvtAv1IntraRefreshType>(PROP_INTRA_REFRESH_DEFAULT);
  svt_config->enc_mode = PROP_ENCMODE_DEFAULT;
  svt_config->frame_rate = 30;
  svt_config->frame_rate_denominator = 0;
  svt_config->frame_rate_numerator = 0;
  svt_config->hierarchical_levels = PROP_HIERARCHICAL_LEVEL_DEFAULT;
  svt_config->pred_structure = PROP_PRED_STRUCTURE_DEFAULT;
  svt_config->scene_change_detection = PROP_SCD_DEFAULT;
  svt_config->rate_control_mode = PROP_RC_MODE_DEFAULT;
  svt_config->target_bit_rate = PROP_BITRATE_DEFAULT;
  svt_config->max_qp_allowed = PROP_QP_MAX_DEFAULT;
  svt_config->min_qp_allowed = PROP_QP_MIN_DEFAULT;
  svt_config->screen_content_mode = 0;
  svt_config->enable_adaptive_quantization = 0;
  svt_config->qp = PROP_QP_DEFAULT;
  svt_config->use_qp_file = 0;
  svt_config->enable_dlf_flag = (PROP_DEBLOCKING_DEFAULT == 1);
  svt_config->film_grain_denoise_strength = 0;
  svt_config->cdef_level = -1;
  svt_config->enable_restoration_filtering = -1;
  svt_config->enable_mfmv = -1;

  // HME parameters
  svt_config->channel_id = 0;
  svt_config->active_channel_count = 1;
  svt_config->recon_enabled = 0;

  // thread affinity
  svt_config->logical_processors = PROP_CORES_DEFAULT;
  svt_config->target_socket = PROP_SOCKET_DEFAULT;
  svt_config->pin_threads = 0;

  // tile based encoding
  svt_config->tile_columns = 0;
  svt_config->tile_rows = 0;
  svt_config->restricted_motion_vector = 0;

  // alt-ref
  svt_config->enable_tf = 1;
  svt_config->enable_overlays = 0;

  // super resolution
  svt_config->superres_mode = SUPERRES_NONE;  // SUPERRES_NONE
  svt_config->superres_denom = 8;
  svt_config->superres_kf_denom = 8;
  svt_config->superres_qthres = 43;

  // latency

  // Annex A
  svt_config->profile = MAIN_PROFILE;
  svt_config->tier = 0;
  svt_config->level = 0;

  svt_config->stat_report = 0;
  svt_config->high_dynamic_range_input = 0;
  svt_config->encoder_bit_depth = 8;
  svt_config->encoder_color_format = static_cast<EbColorFormat>(1);
  svt_config->compressed_ten_bit_format = 0;
  svt_config->use_cpu_flags = CPU_FLAGS_ALL;

  // color description
  svt_config->color_range = 0;
  svt_config->color_primaries = 2;
  svt_config->transfer_characteristics = 2;
  svt_config->matrix_coefficients = 2;
}

SVTAV1Publisher::SVTAV1Publisher()
: logger_(rclcpp::get_logger("SVTAV1Publisher"))
{
}

SVTAV1Publisher::~SVTAV1Publisher()
{
}

std::string SVTAV1Publisher::getTransportName() const
{
  return "svtav1";
}

void SVTAV1Publisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  node_ = node;
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  // for (const ParameterDefinition & pd : kParameters) {
  //   declareParameter(param_base_name, pd);
  // }
}

void SVTAV1Publisher::Initialize(int width, int height) const
{
  this->svt_config = static_cast<EbSvtAv1EncConfiguration *>(malloc(
      sizeof(EbSvtAv1EncConfiguration)));
  if (!this->svt_config) {
    std::cerr << "Error malloc EbSvtAv1EncConfiguration" << std::endl;
    return;
  }
  this->frame_count = 0;

  // Zero-initialize svt_config because svt_av1_enc_init_handle() does not set
  // many fields of svt_config.
  // See https://gitlab.com/AOMediaCodec/SVT-AV1/-/issues/1697.
  memset(this->svt_config, 0, sizeof(EbSvtAv1EncConfiguration));

  EbErrorType res =
    svt_av1_enc_init_handle(&this->svt_encoder, NULL, this->svt_config);
  if (res != EB_ErrorNone) {
    std::cerr << "svt_av1_enc_init_handle failed with error " << res << std::endl;
    free(this->svt_config);
    return;
  }

  // setting configuration here since svt_av1_enc_init_handle overrides it
  set_default_svt_configuration(this->svt_config, width, height);

  res = svt_av1_enc_set_parameter(this->svt_encoder, this->svt_config);
  if (res != EB_ErrorNone) {
    std::cerr << "svt_av1_enc_set_parameter failed with error " << res << std::endl;
    return;
  }

  res = svt_av1_enc_init(this->svt_encoder);

  if (res != EB_ErrorNone) {
    std::cerr << "svt_av1_enc_init failed with error " << res << std::endl;
    return;
  }

  this->input_buf = static_cast<EbBufferHeaderType *>(malloc(sizeof(EbBufferHeaderType)));
  if (!this->input_buf) {
    free(this->svt_config);
    std::cerr << "insufficient resources" << std::endl;
    return;
  }
  this->input_buf->p_buffer = static_cast<uint8_t *>(malloc(sizeof(EbSvtIOFormat)));
  if (!this->input_buf->p_buffer) {
    free(this->svt_config);
    free(this->input_buf);
    std::cerr << "insufficient resources" << std::endl;
    return;
  }
  memset(this->input_buf->p_buffer, 0, sizeof(EbSvtIOFormat));
  this->input_buf->size = sizeof(EbBufferHeaderType);
  this->input_buf->p_app_private = NULL;
  this->input_buf->pic_type = EB_AV1_INVALID_PICTURE;
}

void SVTAV1Publisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  if (this->svt_config == nullptr) {
    this->Initialize(message.width, message.height);
  }

  auto cv_ptr = cv_bridge::toCvCopy(message);
  const cv::Mat & cv_image = cv_ptr->image;
  cv::Mat mat_BGR2YUV_I420;

  cv::cvtColor(cv_image, mat_BGR2YUV_I420, cv::COLOR_RGB2YUV_I420);

  EbErrorType res = EB_ErrorNone;
  EbBufferHeaderType * input_buffer = this->input_buf;
  EbSvtIOFormat * input_picture_buffer =
    reinterpret_cast<EbSvtIOFormat *>(this->input_buf->p_buffer);

  cv::Mat ycrcb_planes[3];  // destination array
  ycrcb_planes[0] = mat_BGR2YUV_I420(cv::Rect(0, 0, cv_image.cols, cv_image.rows));
  ycrcb_planes[1] = mat_BGR2YUV_I420(cv::Rect(0, cv_image.rows, cv_image.cols, cv_image.rows / 4));
  ycrcb_planes[2] =
    mat_BGR2YUV_I420(
    cv::Rect(
      0, cv_image.rows + cv_image.rows / 4, cv_image.cols,
      cv_image.rows / 4));

  input_picture_buffer->width = cv_image.cols;
  input_picture_buffer->height = cv_image.rows;
  input_picture_buffer->bit_depth = EB_EIGHT_BIT;
  input_picture_buffer->color_fmt = EB_YUV420;

  input_picture_buffer->y_stride = cv_image.cols;
  input_picture_buffer->cb_stride = cv_image.cols / 2;
  input_picture_buffer->cr_stride = cv_image.cols / 2;

  input_picture_buffer->luma = ycrcb_planes[0].data;
  input_picture_buffer->cb = ycrcb_planes[1].data;
  input_picture_buffer->cr = ycrcb_planes[2].data;

  input_picture_buffer->luma_ext = NULL;
  input_picture_buffer->cb_ext = NULL;
  input_picture_buffer->cr_ext = NULL;

  input_picture_buffer->origin_x = 0;
  input_picture_buffer->origin_y = 0;

  input_buffer->n_filled_len = mat_BGR2YUV_I420.cols * mat_BGR2YUV_I420.rows;

  /* Fill in Buffers Header control data */
  input_buffer->flags = 0;
  input_buffer->p_app_private = NULL;
  input_buffer->n_alloc_len = 0;
  input_buffer->pts = this->frame_count++;
  input_buffer->pic_type = EB_AV1_INVALID_PICTURE;
  input_buffer->flags = 0;
  input_buffer->metadata = NULL;

  input_buffer->pic_type = EB_AV1_KEY_PICTURE;

  res = svt_av1_enc_send_picture(this->svt_encoder, input_buffer);
  if (res != EB_ErrorNone) {
    std::cerr << "error in sending picture to encoder" << std::endl;
    return;
  }

  EbBufferHeaderType * output_buf = NULL;

  res = svt_av1_enc_get_packet(this->svt_encoder, &output_buf, false);

  uint8_t encode_at_eos = 0;

  if (output_buf != NULL) {
    encode_at_eos =
      ((output_buf->flags & EB_BUFFERFLAG_EOS) == EB_BUFFERFLAG_EOS);
  }

  if (encode_at_eos) {
    std::cerr << "GST_FLOW_EOS" << std::endl;
    return;
  } else if (res == EB_ErrorMax) {
    std::cerr << "encoded failed" << std::endl;
    return;
  } else if (res != EB_NoErrorEmptyQueue && output_buf && output_buf->p_buffer &&  // NOLINT
    (output_buf->n_filled_len > 0))
  {
    sensor_msgs::msg::CompressedImage compressed;
    compressed.data.resize(output_buf->n_filled_len + (3 * sizeof(uint32_t)) + sizeof(uint8_t));

    int message_size = 0;
    memcpy(&compressed.data[message_size], &message.width, sizeof(uint32_t));
    message_size += sizeof(uint32_t);
    memcpy(&compressed.data[message_size], &message.height, sizeof(uint32_t));
    message_size += sizeof(uint32_t);
    memcpy(&compressed.data[message_size], &this->frame_count, sizeof(uint32_t));
    message_size += sizeof(uint32_t);
    compressed.data[message_size] = (input_buffer->pic_type == EB_AV1_KEY_PICTURE);
    message_size += sizeof(uint8_t);
    memcpy(&compressed.data[message_size], output_buf->p_buffer, output_buf->n_filled_len);

    publish_fn(compressed);
  }
}

void SVTAV1Publisher::declareParameter(
  const std::string & base_name,
  const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string transport_name = getTransportName();
  const std::string param_name = base_name + "." + transport_name + "." +
    definition.descriptor.name;
  parameters_.push_back(param_name);

  rclcpp::ParameterValue param_value;

  try {
    param_value = node_->declare_parameter(
      param_name, definition.defaultValue,
      definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    param_value = node_->get_parameter(param_name).get_parameter_value();
  }
}
}  // namespace svtav1_image_transport
