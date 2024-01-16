# Install script for directory: /home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/ImGui/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/JSON/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/GLEW/build/cmake/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/External/GLFW/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Libraries/GuiSupport/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Libraries/SampleProcessing/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/CorePlayground/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/GuiTest/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/SimpleStreamer/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/DepthTester/cmake_install.cmake")
  include("/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/Samples/MultiRecorder/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hello-robot/catkin_ws/src/stretch_ros/vz_optical_remote_vital_sensing/StructureSDK-CrossPlatform-0.9/Builds/linux-release-x86_64/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
