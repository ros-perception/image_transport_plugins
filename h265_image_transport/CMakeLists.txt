cmake_minimum_required(VERSION 3.5)

project(h265_image_transport)

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
# find_package(swscale REQUIRED)
# find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

find_library(AVCODEC_LIBRARY avcodec)


include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/h265_publisher.cpp
  src/h265_subscriber.cpp
  src/manifest.cpp
)

# target_link_libraries(${PROJECT_NAME} /home/ahcorde/libde265/lib/libde265.so)
# target_include_directories(${PROJECT_NAME} PUBLIC /home/ahcorde/libde265/include/)

target_link_libraries(${PROJECT_NAME}
  ${AVCODEC_LIBRARY}
  /usr/lib/x86_64-linux-gnu/libswscale.so
)

ament_target_dependencies(${PROJECT_NAME}
  "cv_bridge"
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
pluginlib_export_plugin_description_file(image_transport h265_plugins.xml)

# add_executable(main
#     src/main.cpp
# )

# target_include_directories(main PUBLIC
#     /home/ahcorde/libde265/include/
# )

# target_link_libraries(main
#     /home/ahcorde/libde265/lib/libde265.so
#     opencv_imgcodecs
#     opencv_highgui
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
