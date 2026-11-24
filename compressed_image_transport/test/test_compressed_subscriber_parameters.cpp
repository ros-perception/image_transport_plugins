// Copyright (c) 2026
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

#include <memory>
#include <string>

#include "compressed_image_transport/compressed_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"

// The transport name ("compressed") is read from the plugin manifest through the ament index,
// so these tests need the package's install space on AMENT_PREFIX_PATH (as under colcon test).
class CompressedSubscriberParameters : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  // SubscriberPlugin::subscribe() expects an already resolved topic name.
  void subscribe(const rclcpp::Node::SharedPtr & node, const std::string & resolved_topic)
  {
    subscriber_.subscribe(
      image_transport::RequiredInterfaces(*node), resolved_topic,
      image_transport::SubscriberPlugin::Callback(
        [](const sensor_msgs::msg::Image::ConstSharedPtr &) {}),
      rclcpp::SensorDataQoS(), rclcpp::SubscriptionOptions());
  }

  compressed_image_transport::CompressedSubscriber subscriber_;
};

TEST_F(CompressedSubscriberParameters, root_namespace)
{
  auto node = std::make_shared<rclcpp::Node>("node");
  subscribe(node, "/image");

  EXPECT_TRUE(node->has_parameter("image.compressed.mode"));
  EXPECT_FALSE(node->has_parameter(".image.compressed.mode"));
}

TEST_F(CompressedSubscriberParameters, namespaced_node)
{
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  subscribe(node, "/ns/image");

  EXPECT_TRUE(node->has_parameter("image.compressed.mode"));
  // The name used before the leading dot was dropped is kept as a deprecated alias.
  EXPECT_TRUE(node->has_parameter(".image.compressed.mode"));
}

TEST_F(CompressedSubscriberParameters, deprecated_name_is_mirrored)
{
  auto node = std::make_shared<rclcpp::Node>("node", "a/b");
  subscribe(node, "/a/b/image");

  node->set_parameter(rclcpp::Parameter(".image.compressed.mode", "gray"));
  EXPECT_EQ(node->get_parameter("image.compressed.mode").as_string(), "gray");

  node->set_parameter(rclcpp::Parameter("image.compressed.mode", "color"));
  EXPECT_EQ(node->get_parameter(".image.compressed.mode").as_string(), "color");
}

TEST_F(CompressedSubscriberParameters, deprecated_name_override_is_honored)
{
  auto node = std::make_shared<rclcpp::Node>(
    "node", "ns",
    rclcpp::NodeOptions().parameter_overrides({{".image.compressed.mode", "gray"}}));
  subscribe(node, "/ns/image");

  EXPECT_EQ(node->get_parameter("image.compressed.mode").as_string(), "gray");
}

TEST_F(CompressedSubscriberParameters, topic_outside_namespace)
{
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  ASSERT_NO_THROW(subscribe(node, "/ab"));

  EXPECT_TRUE(node->has_parameter("ab.compressed.mode"));
}
