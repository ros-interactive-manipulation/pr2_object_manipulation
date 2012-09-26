cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_library(robot_self_filter_color src/self_mask_color.cpp)

#find_package(Boost REQUIRED)
#target_link_libraries(robot_self_filter_color ${Boost_LIBRARIES}) 

rosbuild_add_executable (test_filter_color src/test_filter_color.cpp)
target_link_libraries (test_filter_color robot_self_filter_color)

rosbuild_add_executable(self_filter_color src/self_filter_color.cpp)
target_link_libraries(self_filter_color robot_self_filter_color)
rosbuild_link_boost(self_filter_color signals)
