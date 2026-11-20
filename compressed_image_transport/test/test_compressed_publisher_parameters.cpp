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

#include "compressed_image_transport/compressed_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

class CompressedPublisherParameters : public ::testing::Test
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

  static void advertise(const rclcpp::Node::SharedPtr & node)
  {
    publisher_ = std::make_unique<compressed_image_transport::CompressedPublisher>();
    publisher_->advertise(*node, "image", rclcpp::SensorDataQoS());
  }

  void TearDown() override
  {
    publisher_.reset();
  }

  static std::unique_ptr<compressed_image_transport::CompressedPublisher> publisher_;
};

std::unique_ptr<compressed_image_transport::CompressedPublisher>
CompressedPublisherParameters::publisher_;

TEST_F(CompressedPublisherParameters, root_namespace)
{
  auto node = std::make_shared<rclcpp::Node>("node");
  advertise(node);

  EXPECT_TRUE(node->has_parameter("image.compressed.format"));
  EXPECT_FALSE(node->has_parameter(".image.compressed.format"));
}

TEST_F(CompressedPublisherParameters, namespaced_node)
{
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  advertise(node);

  EXPECT_TRUE(node->has_parameter("image.compressed.format"));
  // The deprecated dot-prefixed name is still declared, but only once.
  EXPECT_TRUE(node->has_parameter(".image.compressed.format"));
  EXPECT_FALSE(node->has_parameter("..image.compressed.format"));
}

TEST_F(CompressedPublisherParameters, nested_namespace_mirrors_deprecated_name)
{
  auto node = std::make_shared<rclcpp::Node>("node", "a/b");
  advertise(node);

  EXPECT_TRUE(node->has_parameter("image.compressed.jpeg_quality"));
  EXPECT_FALSE(node->has_parameter("..image.compressed.jpeg_quality"));

  node->set_parameter(rclcpp::Parameter(".image.compressed.jpeg_quality", 42));
  EXPECT_EQ(node->get_parameter("image.compressed.jpeg_quality").as_int(), 42);
}
