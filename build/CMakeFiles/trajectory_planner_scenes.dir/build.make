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
include CMakeFiles/trajectory_planner_scenes.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/trajectory_planner_scenes.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/trajectory_planner_scenes.dir/flags.make

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o: CMakeFiles/trajectory_planner_scenes.dir/flags.make
CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o: ../src/scenes/BaseScene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o -c /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/BaseScene.cpp

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/BaseScene.cpp > CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.i

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/BaseScene.cpp -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.s

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o: CMakeFiles/trajectory_planner_scenes.dir/flags.make
CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o: ../src/scenes/SimpleScene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o -c /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SimpleScene.cpp

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SimpleScene.cpp > CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.i

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SimpleScene.cpp -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.s

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o: CMakeFiles/trajectory_planner_scenes.dir/flags.make
CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o: ../src/scenes/MultiObstacleScene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o -c /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/MultiObstacleScene.cpp

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/MultiObstacleScene.cpp > CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.i

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/MultiObstacleScene.cpp -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.s

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o: CMakeFiles/trajectory_planner_scenes.dir/flags.make
CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o: ../src/scenes/SlopedScene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o -c /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SlopedScene.cpp

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SlopedScene.cpp > CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.i

CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/src/scenes/SlopedScene.cpp -o CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.s

# Object files for target trajectory_planner_scenes
trajectory_planner_scenes_OBJECTS = \
"CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o" \
"CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o" \
"CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o" \
"CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o"

# External object files for target trajectory_planner_scenes
trajectory_planner_scenes_EXTERNAL_OBJECTS =

libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/src/scenes/BaseScene.cpp.o
libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SimpleScene.cpp.o
libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/src/scenes/MultiObstacleScene.cpp.o
libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/src/scenes/SlopedScene.cpp.o
libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/build.make
libtrajectory_planner_scenes.a: CMakeFiles/trajectory_planner_scenes.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libtrajectory_planner_scenes.a"
	$(CMAKE_COMMAND) -P CMakeFiles/trajectory_planner_scenes.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trajectory_planner_scenes.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/trajectory_planner_scenes.dir/build: libtrajectory_planner_scenes.a

.PHONY : CMakeFiles/trajectory_planner_scenes.dir/build

CMakeFiles/trajectory_planner_scenes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/trajectory_planner_scenes.dir/cmake_clean.cmake
.PHONY : CMakeFiles/trajectory_planner_scenes.dir/clean

CMakeFiles/trajectory_planner_scenes.dir/depend:
	cd /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build /home/mansoor/CLionProjects/TrajectoryOptimization-LocalPlanner/build/CMakeFiles/trajectory_planner_scenes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/trajectory_planner_scenes.dir/depend

