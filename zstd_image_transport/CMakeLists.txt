cmake_minimum_required(VERSION 3.20)

project(zstd_image_transport)

# The C++ standard (cxx_std_20) comes from the
# ament_cmake_ros_core::ament_ros_defaults INTERFACE target, which is linked
# into each target below and propagated to consumers as a minimum standard.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/zlib_cpp.cpp
  src/zstd_publisher.cpp
  src/zstd_subscriber.cpp
  src/manifest.cpp
)

target_link_libraries(${PROJECT_NAME}
  ament_cmake_ros_core::ament_ros_defaults
  ZLIB::ZLIB
  image_transport::image_transport
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
pluginlib_export_plugin_description_file(image_transport zstd_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Compile the codec sources directly into the test so it does not depend on
  # image_transport (and its transitive links) to exercise the compression core.
  ament_add_gtest(test_zstd_roundtrip test/test_zstd_roundtrip.cpp src/zlib_cpp.cpp)
  target_include_directories(test_zstd_roundtrip PRIVATE src)
  target_link_libraries(test_zstd_roundtrip
    ament_cmake_ros_core::ament_ros_defaults
    ZLIB::ZLIB)

  # End-to-end integration over the "zstd" image transport through a real node
  # (requires the installed plugin, discovered via pluginlib at runtime).
  ament_add_gtest(test_zstd_transport test/test_zstd_transport.cpp)
  target_link_libraries(test_zstd_transport
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})
endif()

ament_package()
