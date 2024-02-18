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


#include "svtav1_image_transport/svtav1_subscriber.hpp"

#include <memory>
#include <mutex>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.hpp>

namespace svtav1_image_transport
{
SVTAV1Subscriber::SVTAV1Subscriber()
: logger_(rclcpp::get_logger("SVTAV1Subscriber"))
{
  this->svt_decoder_config =
    static_cast<EbSvtAv1DecConfiguration *>(malloc(sizeof(EbSvtAv1DecConfiguration)));
  if (!this->svt_decoder_config) {
    std::cerr << "Error svt_decoder_config" << std::endl;
    return;
  }

  auto return_error = svt_av1_dec_init_handle(&this->svt_decoder, NULL, this->svt_decoder_config);
  if (return_error != EB_ErrorNone) {
    return_error = svt_av1_dec_deinit_handle(this->svt_decoder);
    return;
  }

  this->svt_decoder_config->operating_point = -1;
  this->svt_decoder_config->output_all_layers = 0;
  this->svt_decoder_config->skip_film_grain = 0;
  this->svt_decoder_config->skip_frames = 0;
  this->svt_decoder_config->frames_to_be_decoded = 0;
  this->svt_decoder_config->compressed_ten_bit_format = 0;
  this->svt_decoder_config->eight_bit_output = 0;

  /* Picture parameters */
  this->svt_decoder_config->max_picture_width = 0;
  this->svt_decoder_config->max_picture_height = 0;
  this->svt_decoder_config->max_bit_depth = EB_EIGHT_BIT;
  this->svt_decoder_config->is_16bit_pipeline = 0;
  this->svt_decoder_config->max_color_format = EB_YUV420;

  // Application Specific parameters
  this->svt_decoder_config->channel_id = 0;
  this->svt_decoder_config->active_channel_count = 1;
  this->svt_decoder_config->stat_report = 0;

  /* Multi-thread parameters */
  this->svt_decoder_config->threads = 10;
  this->svt_decoder_config->num_p_frames = 1;

  return_error = svt_av1_dec_set_parameter(this->svt_decoder, this->svt_decoder_config);
  if (return_error != EB_ErrorNone) {
    std::cerr << "svt_av1_dec_set_parameter failed with error " << return_error << std::endl;
    return;
  }

  return_error = svt_av1_dec_init(this->svt_decoder);
  if (return_error != EB_ErrorNone) {
    std::cerr << "svt_av1_dec_init failed with error " << return_error << std::endl;
    return;
  }

  this->output_buf = static_cast<EbBufferHeaderType *>(malloc(sizeof(EbBufferHeaderType)));
  if (!this->output_buf) {
    std::cerr << "insufficient resources" << std::endl;
    return;
  }

  this->output_buf->p_buffer = static_cast<uint8_t *>(malloc(sizeof(EbSvtIOFormat)));
  if (!this->output_buf->p_buffer) {
    std::cerr << "insufficient resources" << std::endl;
    return;
  }

  buffer = static_cast<EbBufferHeaderType *>(malloc(sizeof(*buffer)));
  if (!buffer) {
    std::cerr << "insufficient resources" << std::endl;
    return;
  }

  buffer->p_buffer = static_cast<uint8_t *>(malloc(sizeof(EbSvtIOFormat)));
  if (!buffer->p_buffer) {
    std::cerr << "insufficient resources" << std::endl;
    return;
  }
  reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->luma = nullptr;
  reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cb = nullptr;
  reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cr = nullptr;
}

std::string SVTAV1Subscriber::getTransportName() const
{
  return "svtav1";
}

void SVTAV1Subscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  node_ = node;
  logger_ = node->get_logger();
  typedef image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
}

void SVTAV1Subscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  std::unique_lock<std::mutex> guard(this->mutex);

  int w = 0;
  int h = 0;
  int frame_count = 0;
  int key_frame = 0;
  int size_message = 0;
  memcpy(&w, &msg->data[size_message], sizeof(uint32_t));
  size_message += sizeof(uint32_t);
  memcpy(&h, &msg->data[size_message], sizeof(uint32_t));
  size_message += sizeof(uint32_t);
  memcpy(&frame_count, &msg->data[size_message], sizeof(uint32_t));
  size_message += sizeof(uint32_t);
  key_frame = msg->data[size_message];
  size_message += sizeof(uint8_t);
  int size =
    (this->svt_decoder_config->max_bit_depth ==
    EB_EIGHT_BIT) ? sizeof(uint8_t) : sizeof(uint16_t);
  size = size * w * h;

  if (reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->luma == NULL) {
    reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->luma =
      reinterpret_cast<uint8_t *>(malloc(size));
    reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cb =
      reinterpret_cast<uint8_t *>(malloc(size >> 2));
    reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cr =
      reinterpret_cast<uint8_t *>(malloc(size >> 2));

    mat_BGR2YUV_I420_decode = cv::Mat(h + h / 2, w, CV_8UC1);
  }

  output_buf->n_filled_len = msg->data.size() - (3 * sizeof(uint32_t) + sizeof(uint8_t));
  const unsigned char * p = msg->data.data() + (3 * sizeof(uint32_t) + sizeof(uint8_t));
  output_buf->p_buffer = (uint8_t *)p;  // NOLINT

  auto return_error = svt_av1_dec_frame(
    this->svt_decoder,
    output_buf->p_buffer,
    output_buf->n_filled_len,
    0);

  if (return_error != EB_ErrorNone) {
    std::cerr << "svt_av1_dec_frame  error" << std::endl;
    return;
  }

  EbAV1StreamInfo stream_info;
  EbAV1FrameInfo frame_info;

  /* FilmGrain module req. even dim. for internal operation */
  EbSvtIOFormat * pic_buffer = reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer);
  pic_buffer->y_stride = w;
  pic_buffer->cb_stride = w / 2;
  pic_buffer->cr_stride = w / 2;
  pic_buffer->width = w;
  pic_buffer->height = h;
  pic_buffer->luma_ext = NULL;
  pic_buffer->cb_ext = NULL;
  pic_buffer->cr_ext = NULL;
  pic_buffer->origin_x = 0;
  pic_buffer->origin_y = 0;
  pic_buffer->bit_depth = this->svt_decoder_config->max_bit_depth;

  try {
    if (svt_av1_dec_get_picture(this->svt_decoder, buffer, &stream_info, &frame_info) !=
      EB_DecNoOutputPicture)
    {
      memcpy(
        mat_BGR2YUV_I420_decode.data,
        reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->luma, size);
      memcpy(
        &mat_BGR2YUV_I420_decode.data[w * h],
        reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cb,
        (size >> 2));
      memcpy(
        &mat_BGR2YUV_I420_decode.data[(w * h) + (size >> 2)],
        reinterpret_cast<EbSvtIOFormat *>(buffer->p_buffer)->cr, (size >> 2));

      cv::Mat rgb;
      cv::cvtColor(mat_BGR2YUV_I420_decode, rgb, cv::COLOR_YUV2RGB_I420);

      std_msgs::msg::Header header;
      header.stamp = this->node_->now();

      auto result = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rgb);
      auto img_msg = result.toImageMsg();
      user_cb(img_msg);
    }
  } catch (...) {
  }
}
}  // namespace svtav1_image_transport
