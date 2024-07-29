// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2016, Google, Inc.
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

#include <limits>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "compressed_depth_image_transport/codec.hpp"
#include "compressed_depth_image_transport/compression_common.hpp"
#include "compressed_depth_image_transport/rvl_codec.hpp"

namespace enc = sensor_msgs::image_encodings;

// Encoding and decoding of compressed depth images.
namespace compressed_depth_image_transport
{

sensor_msgs::msg::Image::SharedPtr decodeCompressedDepthImage(
  const sensor_msgs::msg::CompressedImage & message)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  auto logger = rclcpp::get_logger("compressed_depth_image_transport");
  auto clock = rclcpp::Clock();

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  const size_t split_pos = message.format.find(';');
  const std::string image_encoding = message.format.substr(0, split_pos);
  std::string format;
  // Older version of compressed_depth_image_transport supports only png.
  if (split_pos == std::string::npos) {
    format = "png";
  } else {
    std::string format_ending = message.format.substr(split_pos);
    if (format_ending.find("compressedDepth png") != std::string::npos) {
      format = "png";
    } else if (format_ending.find("compressedDepth rvl") != std::string::npos) {
      format = "rvl";
    } else if (format_ending.find("compressedDepth") != std::string::npos &&  // NOLINT
      format_ending.find("compressedDepth ") == std::string::npos)
    {
      format = "png";
    } else {
      RCLCPP_ERROR(logger, "Unsupported image format: %s", message.format.c_str());
      return sensor_msgs::msg::Image::SharedPtr();
    }
  }

  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(ConfigHeader)) {
    // Read compression type from stream
    ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig),
      message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32) {
      cv::Mat decompressed;
      if (format == "png") {
        try {
          // Decode image data
          decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        } catch (cv::Exception & e) {
          RCLCPP_ERROR(logger, "%s", e.what());
          return sensor_msgs::msg::Image::SharedPtr();
        }
      } else if (format == "rvl") {
        const unsigned char *buffer = imageData.data();

        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        if (rows == 0 || cols == 0) {
          RCLCPP_ERROR_THROTTLE(logger, clock, 1.0,
              "Received malformed RVL-encoded image. Size %ix%i contains zero.", cols, rows);
          return sensor_msgs::msg::Image::SharedPtr();
        }

        // Sanity check - the best compression ratio is 4x; we leave some buffer, so we check
        // whether the output image would not be more than 10x larger than the compressed one.
        // If it is, we probably received corrupted data.
        // The condition should be "numPixels * 2 > compressed.size() * 10"
        // (because each pixel is 2 bytes), but to prevent
        // overflow, we have canceled out the *2 from both sides of the inequality.
        const auto numPixels = static_cast<uint64_t>(rows) * cols;
        if (numPixels > std::numeric_limits<int>::max() ||
          numPixels > static_cast<uint64_t>(imageData.size()) * 5)
        {
          RCLCPP_ERROR_THROTTLE(logger, clock, 1.0,
              "Received malformed RVL-encoded image. It reports size %ux%u.", cols, rows);
          return sensor_msgs::msg::Image::SharedPtr();
        }

        decompressed = cv::Mat(rows, cols, CV_16UC1);
        RvlCodec rvl;
        rvl.DecompressRVL(&buffer[8], decompressed.ptr<uint16_t>(), cols * rows);
      } else {
        return sensor_msgs::msg::Image::SharedPtr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0)) {
        cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
          itDepthImg_end = cv_ptr->image.end<float>();
        cv::MatConstIterator_<uint16_t> itInvDepthImg = decompressed.begin<uint16_t>(),
          itInvDepthImg_end = decompressed.end<uint16_t>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end);
          ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg) {
            *itDepthImg = depthQuantA / static_cast<float>(*itInvDepthImg - depthQuantB);
          } else {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    } else {
      // Decode raw image
      if (format == "png") {
        try {
          cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        } catch (cv::Exception & e) {
          RCLCPP_ERROR(logger, "%s", e.what());
          return sensor_msgs::msg::Image::SharedPtr();
        }
      } else if (format == "rvl") {
        const unsigned char *buffer = imageData.data();
        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        cv_ptr->image = cv::Mat(rows, cols, CV_16UC1);
        RvlCodec rvl;
        rvl.DecompressRVL(&buffer[8], cv_ptr->image.ptr<uint16_t>(), cols * rows);
      } else {
        return sensor_msgs::msg::Image::SharedPtr();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0)) {
        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
  }
  return sensor_msgs::msg::Image::SharedPtr();
}

sensor_msgs::msg::CompressedImage::SharedPtr encodeCompressedDepthImage(
  const sensor_msgs::msg::Image & message,
  const std::string & format,
  double depth_max,
  double depth_quantization,
  int png_level)
{
  // Compressed image message
  sensor_msgs::msg::CompressedImage::SharedPtr compressed(new sensor_msgs::msg::CompressedImage());
  compressed->header = message.header;
  compressed->format = message.encoding;

  auto logger = rclcpp::get_logger("compressed_depth_image_transport");

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);
  int numChannels = enc::numChannels(message.encoding);

  // Image compression configuration
  ConfigHeader compressionConfig;
  compressionConfig.format = INV_DEPTH;

  // Compressed image data
  std::vector<uint8_t> compressedImage;

  // Update ros message format header
  compressed->format += "; compressedDepth " + format;

  // Check input format
  params[0] = cv::IMWRITE_PNG_COMPRESSION;
  params[1] = png_level;

  if ((bitDepth == 32) && (numChannels == 1)) {
    float depthZ0 = depth_quantization;
    float depthMax = depth_max;

    // OpenCV-ROS bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(message);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger, "%s", e.what());
      return sensor_msgs::msg::CompressedImage::SharedPtr();
    }

    const cv::Mat & depthImg = cv_ptr->image;
    size_t rows = depthImg.rows;
    size_t cols = depthImg.cols;

    if ((rows > 0) && (cols > 0)) {
      // Allocate matrix for inverse depth (disparity) coding
      cv::Mat invDepthImg(rows, cols, CV_16UC1);

      // Inverse depth quantization parameters
      float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
      float depthQuantB = 1.0f - depthQuantA / depthMax;

      // Matrix iterators
      cv::MatConstIterator_<float> itDepthImg = depthImg.begin<float>(),
        itDepthImg_end = depthImg.end<float>();
      cv::MatIterator_<uint16_t> itInvDepthImg = invDepthImg.begin<uint16_t>(),
        itInvDepthImg_end = invDepthImg.end<uint16_t>();

      // Quantization
      for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end);
        ++itDepthImg, ++itInvDepthImg)
      {
        // check for NaN & max depth
        if (*itDepthImg < depthMax) {
          *itInvDepthImg = depthQuantA / *itDepthImg + depthQuantB;
        } else {
          *itInvDepthImg = 0;
        }
      }

      // Add coding parameters to header
      compressionConfig.depthParam[0] = depthQuantA;
      compressionConfig.depthParam[1] = depthQuantB;

      // Compress quantized disparity image
      if (format == "png") {
        try {
          // Compress quantized disparity image
          if (cv::imencode(".png", invDepthImg, compressedImage, params)) {
            float cRatio = static_cast<float>(cv_ptr->image.rows * cv_ptr->image.cols *
              cv_ptr->image.elemSize() / compressedImage.size());
            RCLCPP_DEBUG(logger,
                         "Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)",
                cRatio, compressedImage.size());
          } else {
            RCLCPP_ERROR(logger, "cv::imencode (png) failed on input image");
            return sensor_msgs::msg::CompressedImage::SharedPtr();
          }
        } catch (cv::Exception & e) {
          RCLCPP_ERROR(logger, "%s", e.msg.c_str());
          return sensor_msgs::msg::CompressedImage::SharedPtr();
        }
      } else if (format == "rvl") {
        int numPixels = invDepthImg.rows * invDepthImg.cols;
        // In the worst case, RVL compression results in ~1.5x larger data.
        compressedImage.resize(3 * numPixels + 12);
        uint32_t cols = invDepthImg.cols;
        uint32_t rows = invDepthImg.rows;
        memcpy(&compressedImage[0], &cols, 4);
        memcpy(&compressedImage[4], &rows, 4);
        RvlCodec rvl;
        int compressedSize = rvl.CompressRVL(invDepthImg.ptr<uint16_t>(), &compressedImage[8],
            numPixels);
        compressedImage.resize(8 + compressedSize);
      }
    }
  } else if ((bitDepth == 16) && (numChannels == 1)) {  // Raw depth map compression
    // OpenCV-ROS bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(message);
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(logger, "%s", e.msg.c_str());
      return sensor_msgs::msg::CompressedImage::SharedPtr();
    }

    const cv::Mat & depthImg = cv_ptr->image;
    size_t rows = depthImg.rows;
    size_t cols = depthImg.cols;

    if ((rows > 0) && (cols > 0)) {
      uint16_t depthMaxUShort = static_cast<uint16_t>(depth_max * 1000.0f);

      // Matrix iterators
      cv::MatIterator_<uint16_t> itDepthImg = cv_ptr->image.begin<uint16_t>(),
        itDepthImg_end = cv_ptr->image.end<uint16_t>();

      // Max depth filter
      for (; itDepthImg != itDepthImg_end; ++itDepthImg) {
        if (*itDepthImg > depthMaxUShort) {
          *itDepthImg = 0;
        }
      }

      // Compress raw depth image
      if (format == "png") {
        if (cv::imencode(".png", cv_ptr->image, compressedImage, params)) {
          float cRatio = static_cast<float>(cv_ptr->image.rows * cv_ptr->image.cols *
            cv_ptr->image.elemSize() / compressedImage.size());
          RCLCPP_DEBUG(logger,
            "Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)", cRatio,
              compressedImage.size());
        } else {
          RCLCPP_ERROR(logger, "cv::imencode (png) failed on input image");
          return sensor_msgs::msg::CompressedImage::SharedPtr();
        }
      } else if (format == "rvl") {
        int numPixels = cv_ptr->image.rows * cv_ptr->image.cols;
        // In the worst case, RVL compression results in ~1.5x larger data.
        compressedImage.resize(3 * numPixels + 12);
        uint32_t cols = cv_ptr->image.cols;
        uint32_t rows = cv_ptr->image.rows;
        memcpy(&compressedImage[0], &cols, 4);
        memcpy(&compressedImage[4], &rows, 4);
        RvlCodec rvl;
        int compressedSize = rvl.CompressRVL(cv_ptr->image.ptr<uint16_t>(),
            &compressedImage[8], numPixels);
        compressedImage.resize(8 + compressedSize);
      }
    }
  } else {
    RCLCPP_ERROR(logger,
      "Compressed Depth Image Transport - Compression requires single-channel 32bit-floating "
      " point or 16bit raw depth images (input format is: %s).",
        message.encoding.c_str());
    return sensor_msgs::msg::CompressedImage::SharedPtr();
  }

  if (compressedImage.size() > 0) {
    // Add configuration to binary output
    compressed->data.resize(sizeof(ConfigHeader));
    memcpy(&compressed->data[0], &compressionConfig, sizeof(ConfigHeader));

    // Add compressed binary data to messages
    compressed->data.insert(compressed->data.end(), compressedImage.begin(), compressedImage.end());

    return compressed;
  }

  return sensor_msgs::msg::CompressedImage::SharedPtr();
}

}  // namespace compressed_depth_image_transport
