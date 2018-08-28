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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(
    ${PROJECT_NAME}
    SHARED
    src/compressed_depth_publisher.cpp
    src/compressed_depth_subscriber.cpp
    src/manifest.cpp src/codec.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "cv_bridge"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
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
pluginlib_export_plugin_description_file(image_transport
    compressed_depth_plugins.xml)

ament_package()
