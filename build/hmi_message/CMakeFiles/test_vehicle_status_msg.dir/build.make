# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/owen/Documents/HMI_SL4_can_publisher

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/owen/Documents/HMI_SL4_can_publisher/build

# Include any dependencies generated for this target.
include hmi_message/CMakeFiles/test_vehicle_status_msg.dir/depend.make

# Include the progress variables for this target.
include hmi_message/CMakeFiles/test_vehicle_status_msg.dir/progress.make

# Include the compile flags for this target's objects.
include hmi_message/CMakeFiles/test_vehicle_status_msg.dir/flags.make

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/flags.make
hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o: ../hmi_message/test/test_vehicle_status_msg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/owen/Documents/HMI_SL4_can_publisher/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o"
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o -c /home/owen/Documents/HMI_SL4_can_publisher/hmi_message/test/test_vehicle_status_msg.cpp

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.i"
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/owen/Documents/HMI_SL4_can_publisher/hmi_message/test/test_vehicle_status_msg.cpp > CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.i

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.s"
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/owen/Documents/HMI_SL4_can_publisher/hmi_message/test/test_vehicle_status_msg.cpp -o CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.s

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.requires:

.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.requires

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.provides: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.requires
	$(MAKE) -f hmi_message/CMakeFiles/test_vehicle_status_msg.dir/build.make hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.provides.build
.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.provides

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.provides.build: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o


# Object files for target test_vehicle_status_msg
test_vehicle_status_msg_OBJECTS = \
"CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o"

# External object files for target test_vehicle_status_msg
test_vehicle_status_msg_EXTERNAL_OBJECTS =

test/hmi_message/test_vehicle_status_msg: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o
test/hmi_message/test_vehicle_status_msg: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/build.make
test/hmi_message/test_vehicle_status_msg: hmi_message/libhmi_message.so
test/hmi_message/test_vehicle_status_msg: /usr/lib/libgtest.a
test/hmi_message/test_vehicle_status_msg: /usr/lib/libgtest_main.a
test/hmi_message/test_vehicle_status_msg: /usr/lib/x86_64-linux-gnu/libglog.so
test/hmi_message/test_vehicle_status_msg: /usr/lib/x86_64-linux-gnu/libprotobuf.so
test/hmi_message/test_vehicle_status_msg: /usr/lib/x86_64-linux-gnu/libglog.so
test/hmi_message/test_vehicle_status_msg: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/owen/Documents/HMI_SL4_can_publisher/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../test/hmi_message/test_vehicle_status_msg"
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /home/owen/Documents/HMI_SL4_can_publisher /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/drive_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/drive_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/drive
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /home/owen/Documents/HMI_SL4_can_publisher/hmi_message/test/conf /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/conf_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/conf_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/conf
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /home/owen/Documents/HMI_SL4_can_publisher/hmi_message/test/data /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/data_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/data_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/data
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /home/owen/Documents/HMI_SL4_can_publisher/build /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/build_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/build_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/build_dir
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /plusai/map/proto /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/map_proto_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/map_proto_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/map_proto
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/ln --symbolic --force /plusai/plusmap/proto /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/plusmap_proto_test_vehicle_status_msg
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && /bin/mv --no-target-directory /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/plusmap_proto_test_vehicle_status_msg /home/owen/Documents/HMI_SL4_can_publisher/build/test/hmi_message/plusmap_proto
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_vehicle_status_msg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hmi_message/CMakeFiles/test_vehicle_status_msg.dir/build: test/hmi_message/test_vehicle_status_msg

.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/build

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/requires: hmi_message/CMakeFiles/test_vehicle_status_msg.dir/test/test_vehicle_status_msg.cpp.o.requires

.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/requires

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/clean:
	cd /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message && $(CMAKE_COMMAND) -P CMakeFiles/test_vehicle_status_msg.dir/cmake_clean.cmake
.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/clean

hmi_message/CMakeFiles/test_vehicle_status_msg.dir/depend:
	cd /home/owen/Documents/HMI_SL4_can_publisher/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/owen/Documents/HMI_SL4_can_publisher /home/owen/Documents/HMI_SL4_can_publisher/hmi_message /home/owen/Documents/HMI_SL4_can_publisher/build /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message /home/owen/Documents/HMI_SL4_can_publisher/build/hmi_message/CMakeFiles/test_vehicle_status_msg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hmi_message/CMakeFiles/test_vehicle_status_msg.dir/depend

