cmake_minimum_required(VERSION 3.5)

project(theora_image_transport)

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
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED imgproc)
find_package(PkgConfig)
pkg_check_modules(PC_OGG REQUIRED ogg)
pkg_check_modules(PC_THEORA REQUIRED theora)
pkg_check_modules(PC_THEORAENC REQUIRED theoraenc)
pkg_check_modules(PC_THEORADEC REQUIRED theoradec)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Packet.msg"
  DEPENDENCIES
    std_msgs
   ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

set(LIBRARY_NAME ${PROJECT_NAME}_component)

add_library(
  ${LIBRARY_NAME}
  SHARED
  src/theora_publisher.cpp
  src/theora_subscriber.cpp
  src/manifest.cpp
)
target_compile_definitions(${LIBRARY_NAME} PRIVATE
  ${PC_OGG_CFLAGS_OTHER}
  ${PC_THEORA_CFLAGS_OTHER}
  ${PC_THEORAENC_CFLAGS_OTHER}
  ${PC_THEORADEC_CFLAGS_OTHER}
)
target_include_directories(${LIBRARY_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${LIBRARY_NAME}
  opencv_imgproc
  ${PC_OGG_LIBRARIES}
  ${PC_THEORA_LIBRARIES}
  ${PC_THEORAENC_LIBRARIES}
  ${PC_THEORADEC_LIBRARIES}
  "${cpp_typesupport_target}"
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
)

add_executable(ogg_saver src/ogg_saver.cpp)
target_compile_definitions(ogg_saver PRIVATE
  ${PC_OGG_CFLAGS_OTHER}
  ${PC_THEORA_CFLAGS_OTHER}
  ${PC_THEORAENC_CFLAGS_OTHER}
  ${PC_THEORADEC_CFLAGS_OTHER}
)
target_link_libraries(ogg_saver
  ${PC_THEORA_LIBRARY}
  ${PC_OGG_LIBRARY}
  ${PC_THEORAENC_LIBRARIES}
  ${PC_THEORADEC_LIBRARIES}
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  rcutils::rcutils
)

ament_export_dependencies(
  rosidl_default_runtime
  OpenCV
  cv_bridge
  image_transport
  pluginlib
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
)

install(TARGETS ${LIBRARY_NAME} EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${LIBRARY_NAME})

install(TARGETS ogg_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(image_transport theora_plugins.xml)

ament_package()
