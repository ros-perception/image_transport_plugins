cmake_minimum_required(VERSION 3.5)

project(compressed_depth_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_library(
  codecs
  SHARED
  src/codec.cpp
  src/rvl_codec.cpp
)

target_include_directories(
  codecs
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(codecs
  cv_bridge::cv_bridge
  opencv_core
  opencv_imgcodecs
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/compressed_depth_publisher.cpp
  src/compressed_depth_subscriber.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  codecs
  image_transport::image_transport
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  opencv_core
  opencv_imgcodecs
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS codecs
  EXPORT export_codecs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_targets(export_codecs HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge
  rcl_interfaces
  rclcpp
  OpenCV
  sensor_msgs
)

pluginlib_export_plugin_description_file(image_transport
    compressed_depth_plugins.xml)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rvl_codec_test test/rvl_codec_test.cpp)
  target_link_libraries(rvl_codec_test codecs ${PROJECT_NAME})
endif()

ament_package()
