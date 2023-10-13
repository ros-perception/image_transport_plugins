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


#include "h265_image_transport/h265_subscriber.hpp"

#include <avif/avif.h>

#include <memory>
#include <mutex>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>

extern "C" {
  #include <libavcodec/avcodec.h>
  #include <libavutil/opt.h>
  #include <libavutil/imgutils.h>
  #include <libswscale/swscale.h>
}

namespace h265_image_transport
{
H265Subscriber::H265Subscriber()
: logger_(rclcpp::get_logger("H265Subscriber"))
{
  this->codec_ = avcodec_find_decoder(AV_CODEC_ID_H265);
  if (!this->codec_){
    std::cerr << "Could not found codec by given id" << std::endl;
    return;
  }

  this->context_ = avcodec_alloc_context3(this->codec_);
  if (!this->context_){
    std::cerr << "Could not allocate avcodec context" << std::endl;
    return;
  }

  if (avcodec_open2(this->context_, this->codec_, nullptr) < 0){
    std::cerr << "Could not initialize avcodec context" << std::endl;
    return;
  }

  auto desc = av_pix_fmt_desc_get(AV_PIX_FMT_RGB24);
  if (!desc){
    std::cerr << "Can't get descriptor for pixel format" << std::endl;
    return;
  }
  auto bytesPerPixel = av_get_bits_per_pixel(desc) / 8;
  if(!(bytesPerPixel==3 && !(av_get_bits_per_pixel(desc) % 8))) {
    std::cerr << "Unhandled bits per pixel, bad in pix fmt" << std::endl;
    return;
  }
}

std::string H265Subscriber::getTransportName() const
{
  return "h265";
}

void YUVFrameToRGBData(
  SwsContext* swsContext,
  const AVFrame &frame,
  uint8_t *data,
  int bytesPerPixel)
{

  if (!swsContext)
    swsContext = sws_getContext(
      frame.width, frame.height,
      AV_PIX_FMT_YUV420P,
      frame.width, frame.height, // same size, just pix fmt changes
      AV_PIX_FMT_RGB24,
      SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

  uint8_t *outData[1] = {data};
  int outLineSize[1] = {bytesPerPixel * frame.width};
  sws_scale(swsContext, frame.data, frame.linesize, 0, frame.height, outData, outLineSize);
}

void H265Subscriber::subscribeImpl(
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

void H265Subscriber::internalCallback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  AVPacket avPacket;

  AVFrame *frame = av_frame_alloc();
  if(!frame){
    std::cerr << "Could not allocate video frame" << std::endl;
    return;
  }

  std::cout << "msg->data.size() " << msg->data.size() << std::endl;

  int index_data = 0;

  int buf_size;
  std::cout << "index_data_compressed buf_size " << index_data << std::endl;
  memcpy(&buf_size, &msg->data[index_data], sizeof(int));
  index_data += sizeof(int);
  index_data += buf_size;

  std::cout << "buf_size " << buf_size << std::endl;

  int size;
  memcpy(&size, &msg->data[index_data], sizeof(int));
  index_data += sizeof(int);

  av_new_packet(&avPacket, size);
  avPacket.size = size;

  std::cout << "index_data_compressed size " << index_data << std::endl;
  std::cout << "avPacket.size " << avPacket.size << std::endl;
  memcpy(&avPacket.data[0], &msg->data[index_data], size);
  index_data += avPacket.size;

  memcpy(&avPacket.pts, &msg->data[index_data], sizeof(int64_t));
  std::cout << "index_data_compressed pts " << index_data << std::endl;
  index_data += sizeof(int64_t);
  std::cout << "avPacket.pts " << avPacket.pts << std::endl;

  memcpy(&avPacket.dts, &msg->data[index_data], sizeof(int64_t));
  std::cout << "index_data_compressed dts " << index_data << std::endl;
  index_data += sizeof(int64_t);
  std::cout << "avPacket.dts " << avPacket.dts << std::endl;


  memcpy(&avPacket.stream_index, &msg->data[index_data], sizeof(int));
  index_data += sizeof(int);

  if (avPacket.buf == nullptr)
  {
    std::cout << "avPacket.buf nullptr" << std::endl;
  } else {
    std::cout << "avPacket.buf is not nullptr " << avPacket.buf->size << std::endl;
    // std::cout << std::hex << "avPacket.buf is not nullptr avPacket.buf->data " << avPacket.buf->data[0] << std::endl;
    // std::cout << std::hex << "avPacket.buf is not nullptr avPacket.data " << avPacket.data[0] << std::endl;
  }
  // std::cout << std::dec;

  if (avPacket.buf->data == nullptr)
  {
    std::cout << "avPacket.buf->data nullptr" << std::endl;
  } else {
    std::cout << "avPacket.buf->data is not nullptr " << avPacket.buf->size << std::endl;

  }

  // avPacket.buf = av_buffer_alloc(buf_size);
  // if (avPacket.buf == nullptr)
  // {
  //   std::cout << "Error av_buffer_alloc" << std::endl;
  // }
  // // avPacket.buf->size = buf_size;
  // std::cout << "avPacket.buf->size " << avPacket.buf->size << std::endl;

  // memcpy(&avPacket.buf->data, &msg->data[index_data], avPacket.buf->size);
  // std::cout << "index_data_compressed buff data " << index_data << std::endl;
  std::cout << "index_data_compressed index_data " << index_data << std::endl;

  memcpy(&avPacket.flags, &msg->data[index_data], sizeof(int));
  index_data += sizeof(int);

  int side_data_elems = 0;
  memcpy(&side_data_elems, &msg->data[index_data], sizeof(int));
  index_data += sizeof(int);

  for (int i = 0; i < side_data_elems; ++i)
  {
    int side_data_size;
    memcpy(&side_data_size, &msg->data[index_data], sizeof(int));
    index_data += sizeof(int);
    std::cout << "side_data_size " << side_data_size << std::endl;

    int side_data_type;
    memcpy(&side_data_type, &msg->data[index_data], sizeof(int));
    index_data += sizeof(int);
    std::cout << "side_data_type " << side_data_type << std::endl;

    uint8_t *pdata = av_packet_new_side_data(&avPacket,
                                             static_cast<AVPacketSideDataType>(side_data_type),
                                             side_data_size);
    memcpy(pdata, &msg->data[index_data], side_data_size);
    index_data += side_data_size;
  }
  std::cout << "index_data_compressed: antes de duration " << index_data << std::endl;

  memcpy(&avPacket.duration, &msg->data[index_data], sizeof(int64_t));
  index_data += sizeof(int64_t);

  avPacket.pos = -1;
  // memcpy(&avPacket.pos, &msg->data[index_data], sizeof(int64_t));
  // index_data += sizeof(int64_t);

  std::cout << "avPacket.size " << avPacket.size << std::endl;
  std::cout << "avPacket.pts " << avPacket.pts << std::endl;
  std::cout << "avPacket.dts " << avPacket.dts << std::endl;
  std::cout << "avPacket.flags " << avPacket.flags << std::endl;
  std::cout << "avPacket.duration " << avPacket.duration << std::endl;
  std::cout << "avPacket.pos " << avPacket.pos << std::endl;
  std::cout << "avPacket.side_data_elems " << avPacket.side_data_elems << std::endl;

  switch (avcodec_send_packet(this->context_, &avPacket)){
    case 0:
      std::cout << "avcodec_send_packet all good" << std::endl;
      break;
    case AVERROR(EAGAIN):
      std::cout << "avcodec_send_packet EAGAIN" << std::endl;
      break;
    case AVERROR_EOF:
      std::cout << "avcodec_send_packet AVERROR_EOF" << std::endl;
      break;
    case AVERROR(EINVAL):
      std::cout << "avcodec_send_packet AVERROR" << std::endl;
      break;
    case AVERROR(ENOMEM):
      std::cout << " avcodec_send_packetAVERROR" << std::endl;
      break;
    default:
      std::cout << "default" << std::endl;
  }

  // auto result = std::make_shared<sensor_msgs::msg::Image>();

  int ret_receive = -1;
  int n_attemps = 0;
  while (ret_receive != 0) {
    ret_receive = avcodec_receive_frame(this->context_, frame);
    switch (ret_receive) {
      case 0:
        std::cout << "frame " << frame->width << " height " << frame->height << std::endl;
        fflush(stdout);
        // YUVFrameToRGBData(this->swsCtx_, *frame, data, 8);
        // if(avcodec_receive_frame(this->context_, frame) != AVERROR(EAGAIN)){
        //   std::cerr<< "[1-in & N-out] decoders not currently supported!" << std::endl;
        // };
        break;
      case AVERROR(EAGAIN):
        std::cout << "avcodec_receive_frame EAGAIN" << std::endl;
        if (n_attemps++ > 10)
        {
          ret_receive = 0;
          break;
        }
        continue;
      case AVERROR_EOF:
        std::cout << "avcodec_receive_frame AVERROR_EOF" << std::endl;
        break;
      case AVERROR(EINVAL):
        std::cout << "avcodec_receive_frame EINVAL" << std::endl;
        break;
      default:
        std::cout << "avcodec_receive_frame default" << std::endl;
        return;
    }
  }


}
}  // namespace h265_image_transport
