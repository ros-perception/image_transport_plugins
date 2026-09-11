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

#include "avif_image_transport/avif_publisher.hpp"

#include <avif/avif.h>

#include <rclcpp/rclcpp.hpp>

namespace avif_image_transport
{

enum avifParameters
{
  AVIF_QUALITY = 0,
  AVIF_SPEED = 1,
};

const struct ParameterDefinition kParameters[] =
{
  {
    // Quality - AVIF Compression Level from 0 to 100. A higher value means a smaller size.
    ParameterValue(static_cast<int>(75)),
    ParameterDescriptor()
    .set__name("quality")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Quality")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(100)
        .set__step(1)})
  },
  {
    // Speed - 10 - Fastest, 0 - slowest
    ParameterValue(static_cast<int>(5)),
    ParameterDescriptor()
    .set__name("speed")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("Speed")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(10)
        .set__step(1)})
  },
};

AVIFPublisher::AVIFPublisher()
: logger_(rclcpp::get_logger("AVIFPublisher"))
{
  this->encoder_ = avifEncoderCreate();
}

AVIFPublisher::~AVIFPublisher()
{
  if (this->encoder_) {avifEncoderDestroy(this->encoder_);}
}

std::string AVIFPublisher::getTransportName() const
{
  return "avif";
}

void AVIFPublisher::advertiseImpl(
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

  for (const ParameterDefinition & pd : kParameters) {
    declareParameter(param_base_name, pd);
  }
}

AvifImageUniquePtr AVIFPublisher::ConvertToAvif(
  const sensor_msgs::msg::Image & message,
  bool lossless,
  int bit_depth) const
{
  avifImage * result;

  const int width = message.width;
  const int height = message.height;
  if (message.encoding == "mono8") {
    result = avifImageCreateEmpty();
    if (result == nullptr) {
      RCLCPP_DEBUG(logger_, "Not able to create an empty image");
      return nullptr;
    }

    result->width = width;
    result->height = height;
    result->depth = bit_depth;
    result->yuvFormat = AVIF_PIXEL_FORMAT_YUV400;
    result->colorPrimaries = AVIF_COLOR_PRIMARIES_UNSPECIFIED;
    result->transferCharacteristics = AVIF_TRANSFER_CHARACTERISTICS_UNSPECIFIED;
    result->matrixCoefficients = AVIF_MATRIX_COEFFICIENTS_IDENTITY;
    result->yuvRange = AVIF_RANGE_FULL;
    memcpy(&result->yuvPlanes[0], &message.data[0], message.data.size());

    // result->yuvPlanes[0] = message.data;
    result->yuvRowBytes[0] = message.step;
    result->imageOwnsYUVPlanes = AVIF_FALSE;
    return AvifImageUniquePtr(result);
  }

  if (lossless) {
    result =
      avifImageCreate(width, height, bit_depth, AVIF_PIXEL_FORMAT_YUV444);
    if (result == nullptr) {return nullptr;}
    result->colorPrimaries = AVIF_COLOR_PRIMARIES_UNSPECIFIED;
    result->transferCharacteristics = AVIF_TRANSFER_CHARACTERISTICS_UNSPECIFIED;
    result->matrixCoefficients = AVIF_MATRIX_COEFFICIENTS_IDENTITY;
    result->yuvRange = AVIF_RANGE_FULL;
  } else {
    result =
      avifImageCreate(width, height, bit_depth, AVIF_PIXEL_FORMAT_YUV420);
    if (result == nullptr) {return nullptr;}
    result->colorPrimaries = AVIF_COLOR_PRIMARIES_BT709;
    result->transferCharacteristics = AVIF_TRANSFER_CHARACTERISTICS_SRGB;
    result->matrixCoefficients = AVIF_MATRIX_COEFFICIENTS_BT601;
    result->yuvRange = AVIF_RANGE_FULL;
  }

  avifRGBImage rgba;
  avifRGBImageSetDefaults(&rgba, result);
  if (message.encoding == "bgr8") {
    rgba.format = AVIF_RGB_FORMAT_BGR;
  } else if (message.encoding == "bgra8") {
    rgba.format = AVIF_RGB_FORMAT_BGRA;
  }

  rgba.rowBytes = message.step;
  rgba.depth = bit_depth;

  rgba.pixels = reinterpret_cast<uint8_t *>(const_cast<uint8_t *>(&message.data[0]));

  if (avifImageRGBToYUV(result, &rgba) != AVIF_RESULT_OK) {
    avifImageDestroy(result);
    return nullptr;
  }
  return AvifImageUniquePtr(result);
}

void AVIFPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  int bit_depth = 8;
  int speed = node_->get_parameter(parameters_[AVIF_SPEED]).get_value<int>();
  this->encoder_->speed = speed;

  int quality = node_->get_parameter(parameters_[AVIF_QUALITY]).get_value<int>();
  this->encoder_->minQuantizer = encoder_->maxQuantizer =
    (AVIF_QUANTIZER_BEST_QUALITY - AVIF_QUANTIZER_WORST_QUALITY) *
    quality / (100 - 0) +
    AVIF_QUANTIZER_WORST_QUALITY;

  auto avif_image = ConvertToAvif(message, false, bit_depth);
  if (avif_image == nullptr) {
    return;
  }
  avifEncoderAddImage(
    this->encoder_, avif_image.get(), /*durationInTimescale=*/ 1,
    AVIF_ADD_IMAGE_FLAG_SINGLE);

  avifRWData encoded_data = AVIF_DATA_EMPTY;

  avifResult addImageResult = avifEncoderAddImage(
    encoder_,
    avif_image.get(), 1, AVIF_ADD_IMAGE_FLAG_SINGLE);
  if (addImageResult != AVIF_RESULT_OK) {
    // std::cout << "error encoding addImageResult: " << addImageResult << std::endl;
  }

  avifResult finishResult = avifEncoderFinish(encoder_, &encoded_data);
  if (finishResult != AVIF_RESULT_OK) {
    // std::cout << "error encoding finishResult: " << finishResult << std::endl;
  }

  sensor_msgs::msg::CompressedImage compressed;
  compressed.data.resize(encoded_data.size);
  memcpy(&compressed.data[0], encoded_data.data, encoded_data.size);

  publish_fn(compressed);
}

void AVIFPublisher::declareParameter(
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
}  // namespace avif_image_transport
