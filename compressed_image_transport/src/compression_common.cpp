/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 20012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <compressed_image_transport/compression_common.h>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace compressed_image_transport {
    sensor_msgs::ImagePtr decodeCompressedImage(const sensor_msgs::CompressedImageConstPtr &image, int decode_flag) {
        if (!image)
            throw std::runtime_error("Call to decode a compressed image received a NULL pointer.");

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        // Copy message header
        cv_ptr->header = image->header;

        // Decode color/mono image
        cv_ptr->image = cv::imdecode(cv::Mat(image->data), decode_flag);

        // Assign image encoding string
        const size_t split_pos = image->format.find(';');
        if (split_pos == std::string::npos) {
            // Older version of compressed_image_transport does not signal image format
            switch (cv_ptr->image.channels()) {
                case 1:
                    cv_ptr->encoding = enc::MONO8;
                    break;
                case 3:
                    cv_ptr->encoding = enc::BGR8;
                    break;
                default: {
                    std::stringstream ss;
                    ss << "Unsupported number of channels: " << cv_ptr->image.channels();
                    throw std::runtime_error(ss.str());
                }
            }
        } else {
            std::string image_encoding = image->format.substr(0, split_pos);

            cv_ptr->encoding = image_encoding;

            if (enc::isColor(image_encoding)) {
                std::string compressed_encoding = image->format.substr(split_pos);
                bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

                // Revert color transformation
                if (compressed_bgr_image) {
                    // if necessary convert colors from bgr to rgb
                    if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

                    if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

                    if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
                } else {
                    // if necessary convert colors from rgb to bgr
                    if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

                    if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

                    if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
                }
            }
        }

        if ((cv_ptr->image.rows > 0) && (cv_ptr->image.cols > 0))
            return cv_ptr->toImageMsg();
        else {
            std::stringstream ss;
            ss << "Could not extract meaningful image. One of the dimensions was 0. Rows: "
               << cv_ptr->image.rows << ", columns: " << cv_ptr->image.cols << ".";
            throw std::runtime_error(ss.str());
        }
    }
}