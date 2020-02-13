cmake_minimum_required(VERSION 2.8.12)

project(hmi_message_publisher)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(../hmi_message/include)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SRCS
    src/message_handler.cpp
    src/can_node.cpp
    src/config.cpp
)
add_library(${PROJECT_NAME}
    ${SRCS}
)

target_link_libraries(${PROJECT_NAME}
  can_common
  hmi_message
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${ROS_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

#################### tool #######################
create_tool(tools/ros_hmi_message_can_publisher bin/ros_hmi_message_can_publisher)

#################### install ####################

