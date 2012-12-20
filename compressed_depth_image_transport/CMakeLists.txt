cmake_minimum_required(VERSION 2.8)
project(compressed_depth_image_transport)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_transport tf)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/CompressedDepthPublisher.cfg)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

include_directories(${PROJECT_NAME} include cfg/cpp)

add_library(${PROJECT_NAME} src/compressed_depth_publisher.cpp src/compressed_depth_subscriber.cpp src/manifest.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)

class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



