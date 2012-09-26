cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# CMake 2.4.6 doesn't include FindPkgConfig, so we provide our own copy
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

#generate the actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# build is where the UI file outputs (ui_*.h) go.
include_directories( build )

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

qt4_wrap_ui(UIC_FILES
  src/main_frame.ui
)

qt4_wrap_cpp(MOC_FILES
  include/object_segmentation_gui/object_segmentation_rviz_ui.h
)

rosbuild_add_library(object_segmentation_gui_rviz_plugin 
  src/init.cpp
  src/utils.cpp
  src/object_segmentation_display.cpp 
  src/object_segmentation_rviz_ui.cpp
  src/object_segmenter.cpp
  src/table_transform.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)

target_link_libraries(object_segmentation_gui_rviz_plugin ${QT_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(object_segmentation_gui_rviz_plugin thread)

rosbuild_add_executable(tabletop_object_segmentation_gui_node src/tabletop_object_segmentation_gui_node.cpp)

rosbuild_add_executable(ping_object_segmentation_gui src/ping_object_segmentation_gui.cpp)
rosbuild_add_executable(ping_object_segmentation_gui_assembly src/ping_object_segmentation_gui_rgbd_assembly.cpp)
