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

#generate the actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

# This will disable lots of warnings from OgreException.h
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-but-set-parameter" )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# build is where the UI file outputs (ui_*.h) go.
include_directories( build )


pkg_check_modules(OGRE OGRE)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_ui(UIC_FILES
  src/advanced_options_dialog.ui
  src/interactive_manipulation.ui
)

qt4_wrap_cpp(MOC_FILES
  include/pr2_interactive_manipulation/advanced_options_dialog.h
  include/pr2_interactive_manipulation/interactive_manipulation_frontend.h
  include/pr2_interactive_manipulation/camera_focus_frontend.h
  include/pr2_interactive_manipulation/point_head_camera_display.h
  include/pr2_interactive_manipulation/publish_click_camera_display.h
  include/pr2_interactive_manipulation/awesome_camera_display.h
)

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

# rviz plugin
rosbuild_add_library(interactive_manipulation_plugin 
  src/awesome_camera_display.cpp
  src/point_head_camera_display.cpp
  src/point_head_view_controller.cpp
  src/publish_click_camera_display.cpp
  src/publish_click_view_controller.cpp
  src/camera_focus_frontend.cpp
  src/interactive_manipulation_frontend.cpp
  src/interactive_manipulation_frontend_display.cpp
  src/interactive_manipulation_frontend.cpp
  src/advanced_options_dialog.cpp
  src/init.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)
target_link_libraries(interactive_manipulation_plugin 
  default_plugin # from rviz
  ${QT_LIBRARIES}
  ${OGRE_LIBRARIES}
)

# rosbuild_add_executable(grasp_caller src/grasp_caller.cpp)
# target_link_libraries(grasp_caller
#   interactive_manipulation_frontend
# )

