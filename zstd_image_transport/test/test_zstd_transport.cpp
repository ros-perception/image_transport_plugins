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

// Build a bgr8 image with a varying (non-constant) pattern to exercise the codec.
sensor_msgs::msg::Image makeImage(uint32_t w, uint32_t h)
{
  sensor_msgs::msg::Image img;
  img.header.frame_id = "camera";
  img.height = h;
  img.width = w;
  img.encoding = "bgr8";
  img.is_bigendian = 0;
  img.step = w * 3;
  img.data.resize(static_cast<size_t>(img.step) * h);
  for (size_t i = 0; i < img.data.size(); ++i) {
    img.data[i] = static_cast<uint8_t>((i * 131u + 17u) & 0xFFu);
  }
  return img;
}

}  // namespace

class ZstdTransportTest : public ::testing::Test
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

// The zstd transport plugin must be discoverable through pluginlib.
TEST_F(ZstdTransportTest, TransportIsLoadable)
{
  bool found = false;
  for (const auto & name : image_transport::getLoadableTransports()) {
    if (name.find("zstd") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "zstd transport not found among loadable transports";
}

// Full integration over the zstd transport. zstd is lossless, so the received
// image must equal the original exactly (data and layout metadata).
TEST_F(ZstdTransportTest, PublishSubscribeRoundTrip)
{
  auto node = std::make_shared<rclcpp::Node>("zstd_transport_test");

  sensor_msgs::msg::Image::ConstSharedPtr received;
  auto sub = image_transport::create_subscription(
    *node, "test_image",
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {received = msg;},
    "zstd", rclcpp::SystemDefaultsQoS());
  auto pub = image_transport::create_publisher(*node, "test_image", rclcpp::SystemDefaultsQoS());

  const auto original = makeImage(64, 48);

  rclcpp::executors::SingleThreadedExecutor exec;
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!received && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    pub.publish(original);
    exec.spin_node_some(node);
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received) << "no image received through the zstd transport within the timeout";
  EXPECT_EQ(received->width, original.width);
  EXPECT_EQ(received->height, original.height);
  EXPECT_EQ(received->step, original.step);
  EXPECT_EQ(received->encoding, original.encoding);
  EXPECT_EQ(received->data, original.data);  // lossless
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
