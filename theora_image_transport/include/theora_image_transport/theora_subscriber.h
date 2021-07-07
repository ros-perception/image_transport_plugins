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

#include <image_transport/simple_subscriber_plugin.hpp>
#include <theora_image_transport/msg/packet.hpp>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

namespace theora_image_transport {

class TheoraSubscriber : public image_transport::SimpleSubscriberPlugin<theora_image_transport::msg::Packet>
{
public:
  TheoraSubscriber();
  virtual ~TheoraSubscriber();

  virtual std::string getTransportName() const { return "theora"; }

protected:
  // Overridden to bump queue_size, otherwise we might lose headers
  virtual void subscribeImpl(
    rclcpp::Node* node,
    const std::string &base_topic,
    const Callback & callback,
    uint32_t queue_size,
    rmw_qos_profile_t custom_qos);

  // The function that does the actual decompression and calls a user supplied
  // callback with the resulting image
  virtual void internalCallback(const theora_image_transport::msg::Packet::ConstSharedPtr &msg,
                                const Callback& user_cb);

  // Utility functions
  int updatePostProcessingLevel(int level);
  void msgToOggPacket(const theora_image_transport::msg::Packet &msg,
                      ogg_packet &ogg);

  int pplevel_; // Post-processing level
  bool received_header_;
  bool received_keyframe_;
  th_dec_ctx* decoding_context_;
  th_info header_info_;
  th_comment header_comment_;
  th_setup_info* setup_info_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;

  rclcpp::Logger logger_;
};

} //namespace theora_image_transport
