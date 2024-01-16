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

# Utility rule file for MultiRecorderApp.

# Include the progress variables for this target.
include Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/progress.make

Samples/MultiRecorder/CMakeFiles/MultiRecorderApp: ../../Apps/MultiRecorder/MultiRecorder
Samples/MultiRecorder/CMakeFiles/MultiRecorderApp: ../../Apps/MultiRecorder/libStructure.so


../../Apps/MultiRecorder/MultiRecorder: Samples/MultiRecorder/MultiRecorder
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ../../../../Apps/MultiRecorder/MultiRecorder"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder && /usr/local/bin/cmake -E make_directory /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Apps/MultiRecorder
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder && /usr/local/bin/cmake -E copy /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder/MultiRecorder /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Apps/MultiRecorder/MultiRecorder

../../Apps/MultiRecorder/libStructure.so: ../../Libraries/Structure/Linux/x86_64/libStructure.so
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ../../../../Apps/MultiRecorder/libStructure.so"
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder && /usr/local/bin/cmake -E make_directory /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Apps/MultiRecorder
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder && /usr/local/bin/cmake -E copy /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Libraries/Structure/Linux/x86_64/libStructure.so /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Apps/MultiRecorder/libStructure.so

MultiRecorderApp: Samples/MultiRecorder/CMakeFiles/MultiRecorderApp
MultiRecorderApp: ../../Apps/MultiRecorder/MultiRecorder
MultiRecorderApp: ../../Apps/MultiRecorder/libStructure.so
MultiRecorderApp: Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/build.make

.PHONY : MultiRecorderApp

# Rule to build all files generated by this target.
Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/build: MultiRecorderApp

.PHONY : Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/build

Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/clean:
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder && $(CMAKE_COMMAND) -P CMakeFiles/MultiRecorderApp.dir/cmake_clean.cmake
.PHONY : Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/clean

Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/depend:
	cd /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9 /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Samples/MultiRecorder /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64 /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Samples/MultiRecorder/CMakeFiles/MultiRecorderApp.dir/depend
