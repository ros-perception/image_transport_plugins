cmake_minimum_required(VERSION 3.20)

project(depthz_image_transport)

# The codec is performance-critical: never build it unoptimized by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# zstd: use the CMake config package if the distro ships one, otherwise the
# plain library. Prefer the shared target: distro static libzstd.a is not
# built with -fPIC and cannot be linked into this shared plugin library.
find_package(zstd CONFIG QUIET)
if(TARGET zstd::libzstd)
  set(ZSTD_DEPENDENCY zstd::libzstd)
elseif(TARGET zstd::libzstd_shared)
  set(ZSTD_DEPENDENCY zstd::libzstd_shared)
elseif(TARGET zstd::libzstd_static)
  set(ZSTD_DEPENDENCY zstd::libzstd_static)
else()
  find_path(ZSTD_INCLUDE_DIR zstd.h REQUIRED)
  find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
  set(ZSTD_DEPENDENCY ${ZSTD_LIBRARY})
  include_directories(${ZSTD_INCLUDE_DIR})
endif()

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/depth_codec.cpp
  src/depthz_publisher.cpp
  src/depthz_subscriber.cpp
  src/manifest.cpp
)

# The vendored depth codec uses C++20 (std::countr_zero, std::endian).
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME}
  ${ZSTD_DEPENDENCY}
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

# This is a pluginlib-only package: DepthzPublisher/DepthzSubscriber are
# loaded dynamically by class name (see manifest.cpp), never included or
# linked against directly by downstream packages. include/ is therefore not
# installed -- consistent with the sibling compressed_image_transport and
# zstd_image_transport plugin packages, which likewise export nothing. (The
# headers stay in-source for this package's own build/tests, which include
# them via the include_directories(include) above.)
pluginlib_export_plugin_description_file(image_transport depthz_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_codec test/test_depth_codec.cpp)
  target_compile_features(test_depth_codec PRIVATE cxx_std_20)
  target_include_directories(test_depth_codec PRIVATE src)
  target_link_libraries(test_depth_codec ${PROJECT_NAME})
endif()

# Local perf-profiling tool, not part of the package's public surface or its
# CI: exercises DepthzPublisher/DepthzSubscriber end to end through the
# public image_transport API. Off by default; see benchmark/README.md.
option(DEPTHZ_BUILD_BENCHMARK "Build the depthz_image_transport perf benchmark tool" OFF)
if(DEPTHZ_BUILD_BENCHMARK)
  add_executable(benchmark_depthz benchmark/benchmark_depthz.cpp)
  target_compile_features(benchmark_depthz PRIVATE cxx_std_20)
  target_link_libraries(benchmark_depthz
    image_transport::image_transport
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )
  install(TARGETS benchmark_depthz
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
