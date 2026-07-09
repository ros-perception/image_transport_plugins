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
#include <thread>

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

class TheoraTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

// The theora transport plugin must be discoverable through pluginlib.
TEST_F(TheoraTransportTest, TransportIsLoadable)
{
  bool found = false;
  for (const auto & name : image_transport::getLoadableTransports()) {
    if (name.find("theora") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "theora transport not found among loadable transports";
}

// Full integration over the theora transport. Theora is a stateful streaming
// codec (header packets + a keyframe must arrive before a frame decodes), so a
// stream of frames is published. Theora is lossy: only structure is checked.
TEST_F(TheoraTransportTest, PublishSubscribeRoundTrip)
{
  auto node = std::make_shared<rclcpp::Node>("theora_transport_test");

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  sensor_msgs::msg::Image::ConstSharedPtr received;
  auto sub = image_transport::create_subscription(
    *node, "test_image",
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {received = msg;},
    "theora", qos);
  auto pub = image_transport::create_publisher(*node, "test_image", qos);

  const auto original = makeBgr8(64, 48, 40, 90, 160);

  rclcpp::executors::SingleThreadedExecutor exec;

  // Wait for the subscription to be matched before publishing: theora sends its
  // stream headers with the first frame, so a late-matched subscriber would miss
  // them and never be able to decode.
  const auto match_deadline = std::chrono::steady_clock::now() + 10s;
  while (pub.getNumSubscribers() == 0 &&
    std::chrono::steady_clock::now() < match_deadline && rclcpp::ok())
  {
    exec.spin_node_some(node);
    std::this_thread::sleep_for(10ms);
  }

  const auto deadline = std::chrono::steady_clock::now() + 20s;
  while (!received && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    pub.publish(original);
    for (int i = 0; i < 20 && !received; ++i) {
      exec.spin_node_some(node);
      std::this_thread::sleep_for(10ms);
    }
  }

  ASSERT_TRUE(received) << "no image decoded through the theora transport within the timeout";
  EXPECT_EQ(received->width, original.width);
  EXPECT_EQ(received->height, original.height);
  EXPECT_FALSE(received->data.empty());
  EXPECT_FALSE(received->encoding.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
