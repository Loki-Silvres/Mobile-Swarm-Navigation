# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pacman/turtlebot3_ws/src/map_merge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pacman/turtlebot3_ws/build/map_merge

# Include any dependencies generated for this target.
include CMakeFiles/combine_grids.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/combine_grids.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/combine_grids.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/combine_grids.dir/flags.make

CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o: CMakeFiles/combine_grids.dir/flags.make
CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o: /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_compositor.cpp
CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o: CMakeFiles/combine_grids.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pacman/turtlebot3_ws/build/map_merge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o -MF CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o.d -o CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o -c /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_compositor.cpp

CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_compositor.cpp > CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.i

CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_compositor.cpp -o CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.s

CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o: CMakeFiles/combine_grids.dir/flags.make
CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o: /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_warper.cpp
CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o: CMakeFiles/combine_grids.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pacman/turtlebot3_ws/build/map_merge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o -MF CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o.d -o CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o -c /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_warper.cpp

CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_warper.cpp > CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.i

CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/grid_warper.cpp -o CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.s

CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o: CMakeFiles/combine_grids.dir/flags.make
CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o: /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/merging_pipeline.cpp
CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o: CMakeFiles/combine_grids.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pacman/turtlebot3_ws/build/map_merge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o -MF CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o.d -o CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o -c /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/merging_pipeline.cpp

CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/merging_pipeline.cpp > CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.i

CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pacman/turtlebot3_ws/src/map_merge/src/combine_grids/merging_pipeline.cpp -o CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.s

# Object files for target combine_grids
combine_grids_OBJECTS = \
"CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o" \
"CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o" \
"CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o"

# External object files for target combine_grids
combine_grids_EXTERNAL_OBJECTS =

libcombine_grids.a: CMakeFiles/combine_grids.dir/src/combine_grids/grid_compositor.cpp.o
libcombine_grids.a: CMakeFiles/combine_grids.dir/src/combine_grids/grid_warper.cpp.o
libcombine_grids.a: CMakeFiles/combine_grids.dir/src/combine_grids/merging_pipeline.cpp.o
libcombine_grids.a: CMakeFiles/combine_grids.dir/build.make
libcombine_grids.a: CMakeFiles/combine_grids.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pacman/turtlebot3_ws/build/map_merge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libcombine_grids.a"
	$(CMAKE_COMMAND) -P CMakeFiles/combine_grids.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/combine_grids.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/combine_grids.dir/build: libcombine_grids.a
.PHONY : CMakeFiles/combine_grids.dir/build

CMakeFiles/combine_grids.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/combine_grids.dir/cmake_clean.cmake
.PHONY : CMakeFiles/combine_grids.dir/clean

CMakeFiles/combine_grids.dir/depend:
	cd /home/pacman/turtlebot3_ws/build/map_merge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pacman/turtlebot3_ws/src/map_merge /home/pacman/turtlebot3_ws/src/map_merge /home/pacman/turtlebot3_ws/build/map_merge /home/pacman/turtlebot3_ws/build/map_merge /home/pacman/turtlebot3_ws/build/map_merge/CMakeFiles/combine_grids.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/combine_grids.dir/depend

