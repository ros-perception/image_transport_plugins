cmake_minimum_required(VERSION 3.5)

project(zstd_image_transport)

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
find_package(ZLIB REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/zlib_cpp.cpp
  src/zstd_publisher.cpp
  src/zstd_subscriber.cpp
  src/manifest.cpp
)

target_link_libraries(${PROJECT_NAME} ZLIB::ZLIB)

ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
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
pluginlib_export_plugin_description_file(image_transport zstd_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
