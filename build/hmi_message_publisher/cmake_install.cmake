# Install script for directory: /home/owen/Documents/HMI_SL4_can_publisher/hmi_message_publisher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/owen/Documents/HMI_SL4_can_publisher/opt/")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher"
         RPATH "/home/owen/Documents/HMI_SL4_can_publisher/opt//lib:/opt/plusai/lib:/opt/ros/kinetic/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher" TYPE EXECUTABLE FILES "/home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message_publisher/ros_hmi_message_can_publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher"
         OLD_RPATH "/opt/plusai/lib:/opt/ros/kinetic/lib:/home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message_publisher:/home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message:"
         NEW_RPATH "/home/owen/Documents/HMI_SL4_can_publisher/opt//lib:/opt/plusai/lib:/opt/ros/kinetic/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/ros_hmi_message_can_publisher/ros_hmi_message_can_publisher")
    endif()
  endif()
endif()

