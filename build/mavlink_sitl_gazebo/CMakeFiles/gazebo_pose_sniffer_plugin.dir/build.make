# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/nagy/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/nagy/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nagy/test/src/PX4-SITL_gazebo-classic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nagy/test/build/mavlink_sitl_gazebo

# Include any dependencies generated for this target.
include CMakeFiles/gazebo_pose_sniffer_plugin.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/gazebo_pose_sniffer_plugin.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/gazebo_pose_sniffer_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gazebo_pose_sniffer_plugin.dir/flags.make

CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o: CMakeFiles/gazebo_pose_sniffer_plugin.dir/flags.make
CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o: /home/nagy/test/src/PX4-SITL_gazebo-classic/src/gazebo_pose_sniffer_plugin.cpp
CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o: CMakeFiles/gazebo_pose_sniffer_plugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nagy/test/build/mavlink_sitl_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o -MF CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o.d -o CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o -c /home/nagy/test/src/PX4-SITL_gazebo-classic/src/gazebo_pose_sniffer_plugin.cpp

CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nagy/test/src/PX4-SITL_gazebo-classic/src/gazebo_pose_sniffer_plugin.cpp > CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.i

CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nagy/test/src/PX4-SITL_gazebo-classic/src/gazebo_pose_sniffer_plugin.cpp -o CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.s

# Object files for target gazebo_pose_sniffer_plugin
gazebo_pose_sniffer_plugin_OBJECTS = \
"CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o"

# External object files for target gazebo_pose_sniffer_plugin
gazebo_pose_sniffer_plugin_EXTERNAL_OBJECTS =

/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: CMakeFiles/gazebo_pose_sniffer_plugin.dir/src/gazebo_pose_sniffer_plugin.cpp.o
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: CMakeFiles/gazebo_pose_sniffer_plugin.dir/build.make
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstapp-1.0.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libmav_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libnav_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libstd_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libsensor_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libphysics_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.10.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.15.1
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.4.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.8.2
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.11.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.13.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.15.1
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstapp-1.0.so
/home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so: CMakeFiles/gazebo_pose_sniffer_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nagy/test/build/mavlink_sitl_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_pose_sniffer_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gazebo_pose_sniffer_plugin.dir/build: /home/nagy/test/devel/.private/mavlink_sitl_gazebo/lib/libgazebo_pose_sniffer_plugin.so
.PHONY : CMakeFiles/gazebo_pose_sniffer_plugin.dir/build

CMakeFiles/gazebo_pose_sniffer_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gazebo_pose_sniffer_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gazebo_pose_sniffer_plugin.dir/clean

CMakeFiles/gazebo_pose_sniffer_plugin.dir/depend:
	cd /home/nagy/test/build/mavlink_sitl_gazebo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nagy/test/src/PX4-SITL_gazebo-classic /home/nagy/test/src/PX4-SITL_gazebo-classic /home/nagy/test/build/mavlink_sitl_gazebo /home/nagy/test/build/mavlink_sitl_gazebo /home/nagy/test/build/mavlink_sitl_gazebo/CMakeFiles/gazebo_pose_sniffer_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gazebo_pose_sniffer_plugin.dir/depend

