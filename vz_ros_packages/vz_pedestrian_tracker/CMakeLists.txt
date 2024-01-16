cmake_minimum_required(VERSION 3.0.2)
project(vz_pedestrian_tracker)

get_property(GENERATOR_IS_MULTI_CONFIG_VAR GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT GENERATOR_IS_MULTI_CONFIG_VAR AND NOT DEFINED CMAKE_BUILD_TYPE)
    message(STATUS "CMAKE_BUILD_TYPE not defined, 'Release' will be used")
    # Setting CMAKE_BUILD_TYPE as CACHE must go before project(). Otherwise project() sets its value and set() doesn't take an effect
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ...")
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PEDESTRIAN_MODULE_DIR $ENV{PATH_TO_MODULES}/pedestrian_tracker)
set(PEDESTRIAN_LIB_DIR ${PEDESTRIAN_MODULE_DIR}/pedestrian_tracker_demo/cpp)
file(GLOB PEDESTRIAN_LIB_SOURCE ${PEDESTRIAN_LIB_DIR}/src/*.cpp)

set(OpenCV_DIR $ENV{HOME}/intel/openvino_2022/extras/opencv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  vz_face_recognition
)


find_package(OpenCV 4.5.5 REQUIRED COMPONENTS core highgui videoio imgproc gapi)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(${PEDESTRIAN_MODULE_DIR}/thirdparty/gflags ${PEDESTRIAN_MODULE_DIR}/thirdparty/gflags/build/)
add_subdirectory(${PEDESTRIAN_MODULE_DIR}/common/cpp ${PEDESTRIAN_MODULE_DIR}/common/cpp/build/)
add_subdirectory(${PEDESTRIAN_MODULE_DIR}/common/cpp_gapi ${PEDESTRIAN_MODULE_DIR}/common/cpp_gapi/build/)
 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vz_pedestrian_tracker
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs vz_face_recognition
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PEDESTRIAN_LIB_DIR}/include
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  src/${PROJECT_NAME}/person_tracker.cpp
  ${PEDESTRIAN_LIB_SOURCE}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${OpenCV_LIBRARIES}  
  openvino::runtime
  monitors 
  models 
  pipelines
  utils 
  gflags
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(image_converter_node 
  nodes/image_converter_node.cpp
  src/${PROJECT_NAME}/image_converter.cpp
)

target_link_libraries(image_converter_node PRIVATE
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}  
  ${catkin_LIBRARIES} 
)

add_dependencies(image_converter_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)