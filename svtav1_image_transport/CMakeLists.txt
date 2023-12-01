cmake_minimum_required(VERSION 3.16)

project(svtav1_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc videoio)

include_directories(
  include
  /usr/include/svt-av1/
)

add_library(
  ${PROJECT_NAME} SHARED
  src/svtav1_publisher.cpp
  src/svtav1_subscriber.cpp
  src/manifest.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "cv_bridge"
  "image_transport"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
)

target_link_libraries(${PROJECT_NAME}
  /usr/lib/x86_64-linux-gnu/libSvtAv1Enc.so
  /usr/lib/x86_64-linux-gnu/libSvtAv1Dec.so
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
pluginlib_export_plugin_description_file(image_transport svtav1_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
