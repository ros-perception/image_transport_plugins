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

#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

#include <vector>

namespace compressed_image_transport {

using CompressedImage = sensor_msgs::msg::CompressedImage;
using ParameterEvent = rcl_interfaces::msg::ParameterEvent;

class CompressedPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  CompressedPublisher(): logger_(rclcpp::get_logger("CompressedPublisher")) {}
  virtual ~CompressedPublisher() = default;

  virtual std::string getTransportName() const
  {
    return "compressed";
  }

protected:
  // Overridden to set up reconfigure server
  void advertiseImpl(
      rclcpp::Node* node,
      const std::string& base_topic,
      rmw_qos_profile_t custom_qos) override;

  void publish(const sensor_msgs::msg::Image& message,
               const PublishFn& publish_fn) const;

  struct Config {
    // Compression format to use "jpeg" or "png"
    std::string format;

    // PNG Compression Level from 0 to 9.  A higher value means a smaller size.
    // Default to OpenCV default of 3
    int png_level;

    // JPEG Quality from 0 to 100 (higher is better quality).
    // Default to OpenCV default of 95.
    int jpeg_quality;
  };

  Config config_;
  rclcpp::Logger logger_;

private:
  std::vector<std::string> deprecatedParameters_;
  rclcpp::Subscription<ParameterEvent>::SharedPtr parameter_subscription_;

  void onParameterEvent(ParameterEvent::SharedPtr event, std::string full_name);

  void declareParameters(rclcpp::Node* node, const std::string& base_topic);

  template<typename T>
  void declareParameter(rclcpp::Node* node,
                        const std::string &base_name,
                        const std::string &transport_name,
                        const T &default_value,
                        T &out_parameter,
                        const rcl_interfaces::msg::ParameterDescriptor &descriptor);

};

template<typename T>
void CompressedPublisher::declareParameter(rclcpp::Node* node,
                                           const std::string& base_name,
                                           const std::string &transport_name,
                                           const T &default_value,
                                           T &out_parameter,
                                           const rcl_interfaces::msg::ParameterDescriptor &descriptor)
{
  //transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string param_name = base_name + "." + transport_name + "." + descriptor.name;

  try {
    out_parameter = node->declare_parameter(param_name, default_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", descriptor.name.c_str());
    out_parameter = node->get_parameter(param_name).get_value<T>();
  }

  T value_set = out_parameter;

  //deprecated non-scoped parameter name (e.g. image_raw.format)
  const std::string deprecated_name = base_name + "." + descriptor.name;
  deprecatedParameters_.push_back(deprecated_name);

  // transport scoped parameter as default, otherwise we would overwrite
  try {
    out_parameter = node->declare_parameter(deprecated_name, out_parameter, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", descriptor.name.c_str());
    out_parameter = node->get_parameter(deprecated_name).get_value<T>();
  }

  // in case parameter was set through deprecated keep transport scoped parameter in sync
  if(value_set != out_parameter)
    node->set_parameter(rclcpp::Parameter(param_name, out_parameter));

}

} //namespace compressed_image_transport
