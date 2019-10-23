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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED imgproc)
find_package(PkgConfig)
pkg_check_modules(PC_OGG REQUIRED ogg)
pkg_check_modules(PC_THEORA REQUIRED theora)
pkg_check_modules(PC_THEORAENC REQUIRED theoraenc)
pkg_check_modules(PC_THEORADEC REQUIRED theoradec)

set (LIBRARY_NAME ${PROJECT_NAME}_component)

add_library(
  ${LIBRARY_NAME}
  SHARED
  src/theora_publisher.cpp
  src/theora_subscriber.cpp
  src/manifest.cpp
)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${PC_OGG_INCLUDE_DIRS}
  ${PC_THEORA_INCLUDE_DIRS}
  ${PC_THEORAENC_INCLUDE_DIRS}
  ${PC_THEORADEC_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_NAME}
  ${OpenCV_LIBRARIES}
  ${PC_OGG_LIBRARIES}
  ${PC_THEORA_LIBRARIES}
  ${PC_THEORAENC_LIBRARIES}
  ${PC_THEORADEC_LIBRARIES}
)

ament_target_dependencies(${LIBRARY_NAME}
  "image_transport"
  "cv_bridge"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
)

add_definitions(${PC_OGG_CFLAGS_OTHER}
                ${PC_THEORA_CFLAGS_OTHER}
                ${PC_THEORAENC_CFLAGS_OTHER}
                ${PC_THEORADEC_CFLAGS_OTHER})

add_executable(ogg_saver src/ogg_saver.cpp)
target_link_libraries(ogg_saver ${PC_THEORA_LIBRARY} 
                                ${PC_OGG_LIBRARY} 
                                ${OpenCV_LIBRARIES} 
                                ${PC_THEORAENC_LIBRARIES}
                                ${PC_THEORADEC_LIBRARIES})

ament_target_dependencies(ogg_saver
  "rclcpp"
  "rcutils"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Packet.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    sensor_msgs
   ADD_LINTER_TESTS
)

rosidl_target_interfaces(${LIBRARY_NAME}
  ${PROJECT_NAME} "rosidl_typesupport_cpp") 

rosidl_target_interfaces(ogg_saver
  ${PROJECT_NAME} "rosidl_typesupport_cpp") 

ament_export_dependencies(rosidl_default_runtime
  ${OpenCV_LIBRARIES}
  ${PC_OGG_LIBRARIES}
  ${PC_THEORA_LIBRARIES}
  ${PC_THEORAENC_LIBRARIES}
  ${PC_THEORADEC_LIBRARIES}
  ${LIBRARY_NAME})

install(TARGETS ${LIBRARY_NAME} ogg_saver
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

pluginlib_export_plugin_description_file(image_transport theora_plugins.xml)

ament_package()
