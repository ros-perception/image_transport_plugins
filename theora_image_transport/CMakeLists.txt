cmake_minimum_required(VERSION 2.8.3)
project(theora_image_transport)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_transport message_generation tf rosbag)

add_message_files(DIRECTORY msg FILES Packet.msg)

generate_messages(DEPENDENCIES std_msgs)

find_package(PkgConfig)
pkg_check_modules(PC_OGG REQUIRED ogg)
pkg_check_modules(PC_THEORA REQUIRED theora)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/TheoraPublisher.cfg cfg/TheoraSubscriber.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
  ${PC_OGG_INCLUDE_DIRS}
  ${PC_THEORA_INCLUDE_DIRS}
)

link_directories(${PC_OGG_LIBRARY_DIRS} ${PC_THEORA_LIBRARY_DIRS})
add_definitions(${PC_OGG_CFLAGS_OTHER} ${PC_THEORA_CFLAGS_OTHER})

add_library(${PROJECT_NAME} src/theora_publisher.cpp src/theora_subscriber.cpp src/manifest.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${PC_OGG_LIBRARIES}
                                      ${PC_THEORA_LIBRARIES}
#                                      ogg theora theoraenc theoradec
)

class_loader_hide_library_symbols(${PROJECT_NAME})

add_executable(ogg_saver src/ogg_saver.cpp)
include_directories(ogg_saver include cfg/cpp)
target_link_libraries(ogg_saver ${PC_THEORA_LIBRARY} ${PC_OGG_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ogg theora theoraenc theoradec)
add_dependencies(ogg_saver ${PROJECT_NAME}_gencpp)

install(TARGETS ${PROJECT_NAME} ogg_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
