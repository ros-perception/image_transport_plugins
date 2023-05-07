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

#include "compressed_depth_image_transport/compression_common.h"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <rclcpp/node.hpp>

#include <string>
#include <vector>

namespace compressed_depth_image_transport {

using CompressedImage = sensor_msgs::msg::CompressedImage;
using ParameterEvent = rcl_interfaces::msg::ParameterEvent;

class CompressedDepthPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  CompressedDepthPublisher(): logger_(rclcpp::get_logger("CompressedDepthPublisher")) {}
  virtual ~CompressedDepthPublisher() {}

  virtual std::string getTransportName() const
  {
    return "compressedDepth";
  }

protected:
  // Overridden to set up reconfigure server
  void advertiseImpl(
          rclcpp::Node * node,
          const std::string &base_topic,
          rmw_qos_profile_t custom_qos,
          rclcpp::PublisherOptions options) override final;

  void publish(const sensor_msgs::msg::Image& message,
               const PublishFn& publish_fn) const override final;

  rclcpp::Logger logger_;
  rclcpp::Node * node_;
private:
  std::vector<std::string> parameters_;
  std::vector<std::string> deprecatedParameters_;

  rclcpp::Subscription<ParameterEvent>::SharedPtr parameter_subscription_;

  void declareParameter(const std::string &base_name,
                        const ParameterDefinition &definition);

  void onParameterEvent(ParameterEvent::SharedPtr event, std::string full_name, std::string base_name);
};

} //namespace compressed_depth_image_transport
