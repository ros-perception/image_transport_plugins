// Copyright (c) 2026 image_transport_plugins contributors
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
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "compressed_depth_image_transport/compressed_depth_publisher.hpp"

TEST(CompressedDepthPublisherTest, deprecatedParameterOverridesStillApply)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.parameter_overrides({
    rclcpp::Parameter("image_raw.png_level", 7),
    rclcpp::Parameter("image_raw.compressedDepth.depth_max", 12.0),
  });
  const auto node = std::make_shared<rclcpp::Node>("compressed_depth_publisher_test", options);

  // Declaring an alias can immediately enqueue a callback for the scoped parameter.
  const auto declaration_callback = node->add_on_set_parameters_callback(
    [&node](const std::vector<rclcpp::Parameter> & parameters) {
      if (!parameters.empty() && parameters.front().get_name() == "image_raw.format") {
        EXPECT_TRUE(node->has_parameter("image_raw.compressedDepth.png_level"));
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  compressed_depth_image_transport::CompressedDepthPublisher publisher;
  publisher.advertise(node.get(), "/image_raw", rmw_qos_profile_default);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_all(std::chrono::seconds(1));

  EXPECT_EQ(node->get_parameter("image_raw.compressedDepth.png_level").as_int(), 7);
  EXPECT_DOUBLE_EQ(node->get_parameter("image_raw.depth_max").as_double(), 12.0);

  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("image_raw.png_level", 5)).successful);
  executor.spin_all(std::chrono::seconds(1));
  EXPECT_EQ(node->get_parameter("image_raw.compressedDepth.png_level").as_int(), 5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
