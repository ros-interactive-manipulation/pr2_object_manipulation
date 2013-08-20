cmake_minimum_required(VERSION 2.4.6)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# CMake 2.4.6 doesn't include FindPkgConfig, so we provide our own copy
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


# backend node
rosbuild_add_executable(interactive_manipulation_backend_node 
  src/interactive_manipulation_backend.cpp
  src/interactive_manipulation_backend_node.cpp
)

# user interface based on interactive markers
rosbuild_add_executable(interactive_marker_node
  src/interactive_marker_node.cpp
  src/graspable_object_handler.cpp
)

rosbuild_add_roslaunch_check( launch/joy.launch)
rosbuild_add_roslaunch_check( launch/pr2_interactive_manipulation_robot.launch ROBOT=sim)
