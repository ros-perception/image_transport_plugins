// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

namespace
{

// Build a constant-colour bgr8 image with dimensions that are multiples of 16
// (theora encodes in 16x16 macroblocks).
sensor_msgs::msg::Image makeBgr8(uint32_t w, uint32_t h, uint8_t b, uint8_t g, uint8_t r)
{
  sensor_msgs::msg::Image img;
  img.header.frame_id = "camera";
  img.height = h;
  img.width = w;
  img.encoding = "bgr8";
  img.is_bigendian = 0;
  img.step = w * 3;
  img.data.resize(static_cast<size_t>(img.step) * h);
  for (size_t i = 0; i < img.data.size(); i += 3) {
    img.data[i] = b;
    img.data[i + 1] = g;
    img.data[i + 2] = r;
  }
  return img;
}

}  // namespace

// Round-trip through the "theora" transport (TheoraPublisher encode ->
// TheoraSubscriber decode). Theora is a stateful streaming codec: header
// packets must arrive before a keyframe can be decoded, so several frames are
// published. Theora is lossy, so only the cloud structure is checked exactly.
TEST(TheoraRoundTrip, PublishSubscribeDecodesFrame)
{
  auto node = rclcpp::Node::make_shared("test_theora_roundtrip");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  sensor_msgs::msg::Image::ConstSharedPtr received;
  auto sub = image_transport::create_subscription(
    *node, "camera/image",
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {received = msg;},
    "theora", qos);
  auto pub = image_transport::create_publisher(*node, "camera/image", qos);

  const auto original = makeBgr8(64, 48, 40, 90, 160);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto base = node->get_node_base_interface();

  // Publish a stream of frames; the decoder needs the header packets plus a
  // keyframe before it can emit a decoded image.
  const size_t max_frames = 30;
  const size_t max_loops = 50;
  for (size_t frame = 0; frame < max_frames && !received; ++frame) {
    pub.publish(original);
    executor.spin_node_some(base);
    for (size_t loop = 0; !received && loop < max_loops; ++loop) {
      std::this_thread::sleep_for(10ms);
      executor.spin_node_some(base);
    }
  }

  ASSERT_TRUE(received) << "no image decoded through the theora transport";
  EXPECT_EQ(received->width, original.width);
  EXPECT_EQ(received->height, original.height);
  EXPECT_FALSE(received->data.empty());
  EXPECT_FALSE(received->encoding.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
