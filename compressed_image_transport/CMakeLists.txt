cmake_minimum_required(VERSION 3.20)

project(compressed_image_transport)

# The C++ standard (cxx_std_20) comes from the
# ament_cmake_ros_core::ament_ros_defaults INTERFACE target, which is linked
# into each target below and propagated to consumers as a minimum standard.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  SHARED
  src/compressed_publisher.cpp
  src/compressed_subscriber.cpp
  src/manifest.cpp
)

target_link_libraries(${PROJECT_NAME}
  ament_cmake_ros_core::ament_ros_defaults
  ${OpenCV_LIBRARIES}
  image_transport::image_transport
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)
pluginlib_export_plugin_description_file(image_transport compressed_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Round-trips an image through the "compressed" transport end to end
  # (CompressedPublisher encode -> CompressedSubscriber decode) via a real node.
  ament_add_gtest(test_compressed_roundtrip test/test_compressed_roundtrip.cpp)
  target_link_libraries(test_compressed_roundtrip
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})
endif()

ament_package()
