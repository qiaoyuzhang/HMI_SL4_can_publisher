cmake_minimum_required(VERSION 3.5.1)

project(HMI_SL4_can_publisher)
set(CMAKE_VERBOSE_MAKEFILE ON)



#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

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

# ros
if(DEFINED ENV{ROS_ROOT})
  get_filename_component(ROS_INSTALL_DIR $ENV{ROS_ROOT} DIRECTORY)
  get_filename_component(ROS_INSTALL_DIR ${ROS_INSTALL_DIR} DIRECTORY)
  if((NOT EXISTS ${ROS_INSTALL_DIR}) OR (NOT IS_DIRECTORY ${ROS_INSTALL_DIR}))
    message(FATAL_ERROR "ROS install directory ${ROS_INSTALL_DIR} doesn't exist (ROS_ROOT=$ENV{ROS_ROOT})")
  endif()
else()
  message(FATAL_ERROR "Environment variable ROS_ROOT not defined")
endif()
list(APPEND CMAKE_PREFIX_PATH "${ROS_INSTALL_DIR}")

include_directories(SYSTEM ${ROS_INSTALL_DIR}/include)
link_directories(${ROS_INSTALL_DIR}/lib)
set(ROS_LIBRARIES
    roscpp
    cpp_common
    rosconsole
    rosconsole_bridge
    roscpp_serialization
    rostime
    rosbag
    rosbag_storage
    roscpp_serialization
    roslz4
    topic_tools
    message_filters)

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

# now we should have all the global variables set
#message_project_header()

#################### LOAD MODULES

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

# GFlags
find_package(Gflags REQUIRED)
message(STATUS "GFLAGS_LIBRARIES: ${GFLAGS_LIBRARIES}")

# GLog
find_package(Glog REQUIRED)
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

# GTest
find_package(GTest REQUIRED)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS})
message(STATUS "GTEST_LIBRARIES: ${GTEST_LIBRARIES}")

#################### TESTS

add_custom_target(check
    COMMAND ${CMAKE_CTEST_COMMAND})
enable_testing()


#################### COMMON/SHARED DIRECTORIES

#################### LOG SETUP VARS

#################### ADD SUBPROJECT DRECTORIES

add_subdirectory(can_common)
add_subdirectory(hmi_message)
add_subdirectory(hmi_message_publisher)

#################### Gen CMake Configuration File

#message_project_footer()
