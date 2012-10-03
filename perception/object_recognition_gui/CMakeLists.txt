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
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# build is where the UI file outputs (ui_*.h) go.
include_directories( build )

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_ui(UIC_FILES
  src/object_recognition_frame.ui
)

qt4_wrap_cpp(MOC_FILES
  include/object_recognition_gui/object_recognition_rviz_ui.h
  include/object_recognition_gui/mouse_event_signalling_render_panel.h
)

rosbuild_add_library(object_recognition_gui_rviz_plugin 
  src/init.cpp 
  src/object_recognition_display.cpp
  src/object_recognition_rviz_ui.cpp
  src/mouse_event_signalling_render_panel.cpp
  ${UIC_FILES}
  ${MOC_FILES}
  )

target_link_libraries(object_recognition_gui_rviz_plugin ${QT_LIBRARIES})

rosbuild_add_executable(ping_object_recognition_gui src/ping_object_recognition_gui.cpp)