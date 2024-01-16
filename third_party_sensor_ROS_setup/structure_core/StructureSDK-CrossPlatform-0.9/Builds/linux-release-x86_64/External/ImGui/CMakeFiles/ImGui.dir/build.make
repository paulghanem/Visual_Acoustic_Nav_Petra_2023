# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64

# Include any dependencies generated for this target.
include External/ImGui/CMakeFiles/ImGui.dir/depend.make

# Include the progress variables for this target.
include External/ImGui/CMakeFiles/ImGui.dir/progress.make

# Include the compile flags for this target's objects.
include External/ImGui/CMakeFiles/ImGui.dir/flags.make

External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.o: ../../External/ImGui/src/imgui.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/imgui.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/imgui.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui.cpp > CMakeFiles/ImGui.dir/src/imgui.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/imgui.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui.cpp -o CMakeFiles/ImGui.dir/src/imgui.cpp.s

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o: ../../External/ImGui/src/imgui_demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_demo.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/imgui_demo.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_demo.cpp > CMakeFiles/ImGui.dir/src/imgui_demo.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/imgui_demo.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_demo.cpp -o CMakeFiles/ImGui.dir/src/imgui_demo.cpp.s

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o: ../../External/ImGui/src/imgui_draw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_draw.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/imgui_draw.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_draw.cpp > CMakeFiles/ImGui.dir/src/imgui_draw.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/imgui_draw.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_draw.cpp -o CMakeFiles/ImGui.dir/src/imgui_draw.cpp.s

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o: ../../External/ImGui/src/imgui_widgets.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_widgets.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_widgets.cpp > CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/imgui_widgets.cpp -o CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.s

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o: ../../External/ImGui/src/examples/imgui_impl_glfw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_glfw.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_glfw.cpp > CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_glfw.cpp -o CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.s

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o: External/ImGui/CMakeFiles/ImGui.dir/flags.make
External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o: ../../External/ImGui/src/examples/imgui_impl_opengl3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o -c /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_opengl3.cpp

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.i"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_opengl3.cpp > CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.i

External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.s"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui/src/examples/imgui_impl_opengl3.cpp -o CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.s

# Object files for target ImGui
ImGui_OBJECTS = \
"CMakeFiles/ImGui.dir/src/imgui.cpp.o" \
"CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o" \
"CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o" \
"CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o" \
"CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o" \
"CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o"

# External object files for target ImGui
ImGui_EXTERNAL_OBJECTS =

External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/imgui.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/imgui_demo.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/imgui_draw.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/imgui_widgets.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_glfw.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/src/examples/imgui_impl_opengl3.cpp.o
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/build.make
External/ImGui/libImGui.a: External/ImGui/CMakeFiles/ImGui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libImGui.a"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && $(CMAKE_COMMAND) -P CMakeFiles/ImGui.dir/cmake_clean_target.cmake
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ImGui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
External/ImGui/CMakeFiles/ImGui.dir/build: External/ImGui/libImGui.a

.PHONY : External/ImGui/CMakeFiles/ImGui.dir/build

External/ImGui/CMakeFiles/ImGui.dir/clean:
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui && $(CMAKE_COMMAND) -P CMakeFiles/ImGui.dir/cmake_clean.cmake
.PHONY : External/ImGui/CMakeFiles/ImGui.dir/clean

External/ImGui/CMakeFiles/ImGui.dir/depend:
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9 /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/External/ImGui /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64 /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui/CMakeFiles/ImGui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : External/ImGui/CMakeFiles/ImGui.dir/depend

