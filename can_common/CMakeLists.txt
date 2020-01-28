cmake_minimum_required(VERSION 2.8.3)
project(can_common)

add_definitions(-Wall -std=c++14)

list(APPEND CMAKE_MODULE_PATH "/opt/plusai/var/cmake/Modules")
find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME})


include_directories(
        include
        ${catkin_INCLUDE_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

# TODO: QNX, CAN interface on QNX is different from Linux, need a seperate PR for QNX CAN
if(CMAKE_SYSTEM_NAME MATCHES "QNX")
    list(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/can_connection.cpp")
    set(EXTRA_LIBRARIES socket)
else()
    set(EXTRA_LIBRARIES)
endif()

add_library(${PROJECT_NAME}
        ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${EXTRA_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/can_common/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
