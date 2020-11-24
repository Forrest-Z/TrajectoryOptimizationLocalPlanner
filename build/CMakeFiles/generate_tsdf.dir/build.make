# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
CMAKE_COMMAND = /opt/clion-2019.1.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.1.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build

# Include any dependencies generated for this target.
include CMakeFiles/generate_tsdf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/generate_tsdf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/generate_tsdf.dir/flags.make

CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o: CMakeFiles/generate_tsdf.dir/flags.make
CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o: ../src/generate_tsdf.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o -c /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/generate_tsdf.cpp

CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/generate_tsdf.cpp > CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.i

CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/generate_tsdf.cpp -o CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.s

# Object files for target generate_tsdf
generate_tsdf_OBJECTS = \
"CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o"

# External object files for target generate_tsdf
generate_tsdf_EXTERNAL_OBJECTS =

generate_tsdf: CMakeFiles/generate_tsdf.dir/src/generate_tsdf.cpp.o
generate_tsdf: CMakeFiles/generate_tsdf.dir/build.make
generate_tsdf: libtrajectory_planner_scenes.a
generate_tsdf: libtrajectory_planner_mapping.a
generate_tsdf: /home/mansoor/Downloads/glog-master/build/libglog.a
generate_tsdf: /home/mansoor/catkin_ws_voxbloxpp/devel/lib/libgflags.so.2.2.1
generate_tsdf: /usr/lib/x86_64-linux-gnu/libprotobuf.so
generate_tsdf: libtrajectory_planner_proto.a
generate_tsdf: CMakeFiles/generate_tsdf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable generate_tsdf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/generate_tsdf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/generate_tsdf.dir/build: generate_tsdf

.PHONY : CMakeFiles/generate_tsdf.dir/build

CMakeFiles/generate_tsdf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/generate_tsdf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/generate_tsdf.dir/clean

CMakeFiles/generate_tsdf.dir/depend:
	cd /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles/generate_tsdf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/generate_tsdf.dir/depend

