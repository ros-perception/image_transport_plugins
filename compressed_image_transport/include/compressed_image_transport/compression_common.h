/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON
#define COMPRESSED_IMAGE_TRANSPORT_COMPRESSION_COMMON

namespace compressed_image_transport
{

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1,
  JPEG = 0,
  PNG = 1,
  TIFF = 2,
};

// Parameters
// Note - what is below would be moved to separate file, e.g. `compressed_publisher_cfg.h`

enum compressedParameters
{
  FORMAT = 0,
  PNG_LEVEL,
  JPEG_QUALITY,
  TIFF_RESOLUTION_UNIT,
  TIFF_XDPI,
  TIFF_YDPI
};

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParameterValue = rclcpp::ParameterValue;

struct ParameterDefinition
{
  const ParameterValue defaultValue;
  const ParameterDescriptor descriptor;
};

const struct ParameterDefinition kParameters[] =
{
  { //FORMAT - Compression format to use "jpeg", "png" or "tiff".
    ParameterValue("jpeg"),
    ParameterDescriptor()
      .set__name("format")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("Compression method")
      .set__read_only(false)
      .set__additional_constraints("Supported values: [jpeg, png, tiff]")
  },
  { //PNG_LEVEL - PNG Compression Level from 0 to 9.  A higher value means a smaller size.
    ParameterValue((int)3), //Default to OpenCV default of 3
    ParameterDescriptor()
      .set__name("png_level")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("Compression level for PNG format")
      .set__read_only(false)
      .set__integer_range(
        {rcl_interfaces::msg::IntegerRange()
          .set__from_value(0)
          .set__to_value(9)
          .set__step(1)})
  },
  { //JPEG_QUALITY - JPEG Quality from 0 to 100 (higher is better quality).
    ParameterValue((int)95), //Default to OpenCV default of 95.
    ParameterDescriptor()
      .set__name("jpeg_quality")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("Image quality for JPEG format")
      .set__read_only(false)
      .set__integer_range(
        {rcl_interfaces::msg::IntegerRange()
          .set__from_value(1)
          .set__to_value(100)
          .set__step(1)})
  },
  { //TIFF_RESOLUTION_UNIT - TIFF resolution unit, can be one of "none", "inch", "centimeter".
    ParameterValue("inch"),
    ParameterDescriptor()
      .set__name("tiff.res_unit")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("tiff resolution unit")
      .set__read_only(false)
      .set__additional_constraints("Supported values: [none, inch, centimeter]")
  },
  { //TIFF_XDPI
    ParameterValue((int)-1),
    ParameterDescriptor()
      .set__name("tiff.xdpi")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("tiff xdpi")
      .set__read_only(false)
  },
  { //TIFF_YDPI
    ParameterValue((int)-1),
    ParameterDescriptor()
      .set__name("tiff.ydpi")
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
      .set__description("tiff ydpi")
      .set__read_only(false)
  }
};

} //namespace compressed_image_transport

#endif
