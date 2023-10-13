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

#include "h265_image_transport/h265_publisher.hpp"

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <libde265/de265.h>
#include <libde265/en265.h>
#include <libde265/image.h>

#include <cv_bridge/cv_bridge.hpp>

namespace h265_image_transport
{

// enum hParameters
// {
// };

// const struct ParameterDefinition kParameters[] =
// {
// };

H265Publisher::H265Publisher()
: logger_(rclcpp::get_logger("H265Publisher"))
{
  de265_init();
  this->ectx_ = en265_new_encoder();
  en265_start_encoder(this->ectx_, 0);
}

H265Publisher::~H265Publisher()
{
}

std::string H265Publisher::getTransportName() const
{
  return "h265";
}

void H265Publisher::advertiseImpl(
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

void H265Publisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // if (de265_get_image_width(img, 0) == 0)
  // {
    auto img = new struct de265_image() ;
    img->alloc_image(
      message.width, message.height, de265_chroma_444, NULL, false,
      NULL, /*NULL,*/ 0, NULL, false);
    std::cout << "alloc_image " << std::endl;
  // }

  std::cout << "de265_get_image_width(img, 0) " << de265_get_image_width(img, 0) << std::endl;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(message, message.encoding);
    std::cout << "cv_ptr->image: " << cv_ptr->image.rows << " " << cv_ptr->image.cols << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->logger_, "cv_bridge exception: %s", e.what());
    return;
  }

  uint8_t * p;
  int stride;

  for (int c = 0; c < 3; c++) {
    int h265channel;
    switch (c) {
      case 0: h265channel = 2; break; // R
      case 1: h265channel = 0; break; // G
      case 2: h265channel = 1; break; // B
    }

    p = img->get_image_plane(h265channel);
    stride = img->get_image_stride(h265channel);

    cv::Mat ch1;
    cv::extractChannel(cv_ptr->image, ch1, c); // extract specific channel

    for (int y = 0; y < cv_ptr->image.rows; y++) {
      memcpy(p, &ch1.data[y * cv_ptr->image.cols], cv_ptr->image.cols);
      p += stride;
    }
  }

  std::cout << "image filled" << std::endl;

  en265_push_image(this->ectx_, img);
  std::cout << "en265_push_image" << std::endl;

  en265_encode(this->ectx_);
  std::cout << "en265_encode " << en265_number_of_queued_packets(this->ectx_) << std::endl;

  sensor_msgs::msg::CompressedImage compressed;

  int size = 0;
  std::vector<en265_packet *> packets;

  // if (en265_number_of_queued_packets(this->ectx_) < 7)
  // {
  //   return;
  // }


  while (en265_number_of_queued_packets(this->ectx_) > 0) {
    en265_packet * pck = en265_get_packet(this->ectx_, 0);
    size += pck->length;
    packets.push_back(pck);
  }
  std::cout << "pckt size " << size << std::endl;

  compressed.data.resize(size);
  int index = 0;
  for (size_t i = 0; i < packets.size(); ++i)
  {
    memcpy(&compressed.data[index], packets[i]->data, packets[i]->length);
    index += packets[i]->length;
    en265_free_packet(this->ectx_, packets[i]);
  }
  std::cout << "cpied " << size << std::endl;

  publish_fn(compressed);
  std::cout << "published " << std::endl;

  // img->release();
}

void H265Publisher::declareParameter(
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
}  // namespace h265_image_transport
