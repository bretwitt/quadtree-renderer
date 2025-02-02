# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /home/bret/anaconda3/bin/cmake

# The command to remove a file.
RM = /home/bret/anaconda3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bret/Desktop/QuadTreeRenderer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bret/Desktop/QuadTreeRenderer/build

# Include any dependencies generated for this target.
include CMakeFiles/HeightfieldRenderer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/HeightfieldRenderer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/HeightfieldRenderer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HeightfieldRenderer.dir/flags.make

CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/main.cpp
CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/main.cpp

CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/main.cpp > CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/main.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.s

CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/Application.cpp
CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/Application.cpp

CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/Application.cpp > CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/Application.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.s

CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/Shader.cpp
CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/Shader.cpp

CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/Shader.cpp > CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/Shader.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.s

CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/Heightfield.cpp
CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/Heightfield.cpp

CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/Heightfield.cpp > CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/Heightfield.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.s

CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/Camera.cpp
CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/Camera.cpp

CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/Camera.cpp > CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/Camera.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.s

CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o: CMakeFiles/HeightfieldRenderer.dir/flags.make
CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o: /home/bret/Desktop/QuadTreeRenderer/src/QuadtreeRenderer.cpp
CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o: CMakeFiles/HeightfieldRenderer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o -MF CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o.d -o CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o -c /home/bret/Desktop/QuadTreeRenderer/src/QuadtreeRenderer.cpp

CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bret/Desktop/QuadTreeRenderer/src/QuadtreeRenderer.cpp > CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.i

CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bret/Desktop/QuadTreeRenderer/src/QuadtreeRenderer.cpp -o CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.s

# Object files for target HeightfieldRenderer
HeightfieldRenderer_OBJECTS = \
"CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o" \
"CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o" \
"CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o" \
"CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o" \
"CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o" \
"CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o"

# External object files for target HeightfieldRenderer
HeightfieldRenderer_EXTERNAL_OBJECTS =

HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/main.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/Application.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/Shader.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/Heightfield.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/Camera.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/src/QuadtreeRenderer.cpp.o
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/build.make
HeightfieldRenderer: /usr/lib/x86_64-linux-gnu/libGL.so
HeightfieldRenderer: libglad.a
HeightfieldRenderer: CMakeFiles/HeightfieldRenderer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable HeightfieldRenderer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HeightfieldRenderer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HeightfieldRenderer.dir/build: HeightfieldRenderer
.PHONY : CMakeFiles/HeightfieldRenderer.dir/build

CMakeFiles/HeightfieldRenderer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HeightfieldRenderer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HeightfieldRenderer.dir/clean

CMakeFiles/HeightfieldRenderer.dir/depend:
	cd /home/bret/Desktop/QuadTreeRenderer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bret/Desktop/QuadTreeRenderer /home/bret/Desktop/QuadTreeRenderer /home/bret/Desktop/QuadTreeRenderer/build /home/bret/Desktop/QuadTreeRenderer/build /home/bret/Desktop/QuadTreeRenderer/build/CMakeFiles/HeightfieldRenderer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HeightfieldRenderer.dir/depend

