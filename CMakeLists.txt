cmake_minimum_required(VERSION 3.5.1)

project(hmi_message_publisher)
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_VERBOSE_MAKEFILE ON)

# plusai cmake module path
set(PLUSAI_CMAKE_MODULE_PATH "/opt/plusai/var/cmake/Modules" CACHE PATH "")

# cmake module path
list(APPEND CMAKE_MODULE_PATH ${PLUSAI_CMAKE_MODULE_PATH})
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

# include plusai_util.cmake
include(${PLUSAI_CMAKE_MODULE_PATH}/plusai_util.cmake RESULT_VARIABLE PLUSAI_UTIL_CMAKE)
if ("${PLUSAI_UTIL_CMAKE}" STREQUAL "NOTFOUND")
    message(FATAL_ERROR "Unable to load plusai util cmake")
endif()

# by default build debug version
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif(NOT DEFINED CMAKE_BUILD_TYPE)

# set drive root from environment, else use default
if(NOT DEFINED DRIVE_ROOT)
    if(DEFINED ENV{DRIVE_ROOT})
        set(DRIVE_ROOT "$ENV{DRIVE_ROOT}")
    else()
        set(DRIVE_ROOT "/opt/plusai")
    endif()
endif()
message(STATUS "DRIVE_ROOT: ${DRIVE_ROOT}")

# Global flag to cause add_library to create shared libraries if ON.
set(BUILD_SHARED_LIBS ON)

string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_DIR)

SET(CMAKE_SKIP_BUILD_RPATH FALSE)

option(STAGING "Produce a build suitable for packaging." OFF)
if(STAGING)
  message(STATUS "************** STAGING ****************")
  # Install with rpath for packaging
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/opt/staging")
  set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/plusai")
  set(CMAKE_INSTALL_RPATH "${CPACK_PACKAGING_INSTALL_PREFIX}/lib")
  set(RUNTIME_ROOT "${CPACK_PACKAGING_INSTALL_PREFIX}")
else()
  # Install with rpath for local install dir
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/opt/${BUILD_TYPE_DIR}")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  set(RUNTIME_ROOT "${CMAKE_INSTALL_PREFIX}")
endif()

# set(CMAKE_DEBUG_TARGET_PROPERTIES)

# to include the header based on the full path.
list(APPEND CMAKE_PREFIX_PATH "${DRIVE_ROOT}")
find_package(drive REQUIRED)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${drive_INCLUDE_DIRS}/common)
include_directories(${drive_INCLUDE_DIRS}/proto)
message(STATUS "DRIVE INCLUDE: ${drive_INCLUDE_DIRS}")

# default installed headers and libraries
link_directories(${drive_LIBRARY_DIRS})
message(STATUS "DRIVE_LIBRARIES: ${drive_LIBRARY_DIRS}")

# Opitonal build profiling
option(BUILD_PROFILING "Enables build profile logging" OFF)
set(PROFILING_COMPILE "/usr/bin/time --format 'compile:%C;%U;%S;%e;%I;%O'")
set(PROFILING_LINK "/usr/bin/time --format 'link:%C;%U;%S;%e;%I;%O'")
message(STATUS "BUILD_PROFILING is ${BUILD_PROFILING}")

# log the final launch commands
get_property(GLOBAL_RULE_LAUNCH_COMPILE GLOBAL PROPERTY RULE_LAUNCH_COMPILE)
message(STATUS "RULE_LAUNCH_COMPILE: ${GLOBAL_RULE_LAUNCH_COMPILE}")
get_property(GLOBAL_RULE_LAUNCH_LINK GLOBAL PROPERTY RULE_LAUNCH_LINK)
message(STATUS "RULE_LAUNCH_LINK: ${GLOBAL_RULE_LAUNCH_LINK}")

# Use ld.gold
set(LINKER_EXTRA_FLAGS "-fuse-ld=gold -Wl,--threads -Wl,--disable-new-dtags -Wl,--compress-debug-sections=zlib -Wl,-O2")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_EXTRA_FLAGS}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${LINKER_EXTRA_FLAGS}")
message(STATUS "CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} ${LINKER_EXTRA_FLAGS}")
message(STATUS "CMAKE_STATIC_LINKER_FLAGS: ${CMAKE_STATIC_LINKER_FLAGS}")

# log the final launch commands
get_property(GLOBAL_RULE_LAUNCH_COMPILE GLOBAL PROPERTY RULE_LAUNCH_COMPILE)
message(STATUS "RULE_LAUNCH_COMPILE: ${GLOBAL_RULE_LAUNCH_COMPILE}")
get_property(GLOBAL_RULE_LAUNCH_LINK GLOBAL PROPERTY RULE_LAUNCH_LINK)
message(STATUS "RULE_LAUNCH_LINK: ${GLOBAL_RULE_LAUNCH_LINK}")

#################### LOAD MODULES

# Boost
find_package(Boost REQUIRED COMPONENTS system)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_VERSION: ${EIGEN3_VERSION}")

# Protobuf
include(FindProtobuf)
find_package(Protobuf REQUIRED 3.2.0)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIRS})
link_directories(${PROTOBUF_LIB_DIR})
message(STATUS "PROTOBUF_FOUND: ${PROTOBUF_FOUND}")
message(STATUS "PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")
message(STATUS "PROTOBUF_INCLUDE_DIRS: ${PROTOBUF_INCLUDE_DIRS}")
message(STATUS "PROTOBUF_LIB_DIR: ${PROTOBUF_LIB_DIR}")
message(STATUS "PROTOBUF_LIBRARIES: ${PROTOBUF_LIBRARIES}")
message(STATUS "PROTOBUF_LIBRARY: ${PROTOBUF_LIBRARY}")
message(STATUS "PROTOBUF_PROTOC_EXECUTABLE: ${PROTOBUF_PROTOC_EXECUTABLE}")
message(STATUS "PROTOBUF_PROTOC_LIBRARIES: ${PROTOBUF_PROTOC_LIBRARIES}")
message(STATUS "PROTOBUF_PROTOC_LIBRARY: ${PROTOBUF_PROTOC_LIBRARY}")

# GLog
find_package(Glog REQUIRED)
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

# GTest
find_package(GTest REQUIRED)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS})
message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# include_directories(../hmi_message/include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_common
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  can_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")

add_library(${PROJECT_NAME}_lib
  ${SOURCES}
)

add_executable(${PROJECT_NAME}_node
  tools/ros_hmi_message_can_publisher.cpp
)

add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  drive_common
  drive_common_proto
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  drive_common
  drive_common_proto
)

# target_link_directories(${PROJECT_NAME}_node ${drive_LIBRARY_DIRS})

#################### tool #######################
# create_tool(tools/ros_hmi_message_can_publisher bin/ros_hmi_message_can_publisher)

#################### install ####################
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#################### test #######################
# if (CATKIN_ENABLE_TESTING)
#   add_subdirectory(tests)
# endif()
