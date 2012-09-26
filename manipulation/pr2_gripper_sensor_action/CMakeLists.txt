cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)



rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(pr2_gripper_sensor_action src/pr2_gripper_sensor_action.cpp)
rosbuild_add_executable(pr2_gripper_findContact_action src/pr2_gripper_findContact_action.cpp)
rosbuild_add_executable(pr2_gripper_slipServo_action src/pr2_gripper_slipServo_action.cpp)
rosbuild_add_executable(pr2_gripper_event_detector_action     src/pr2_gripper_event_detector_action.cpp)
rosbuild_add_executable(pr2_gripper_forceServo_action     src/pr2_gripper_forceServo_action.cpp)
rosbuild_add_executable(positionActionTest              src/positionActionTest.cpp)
rosbuild_add_executable(findContactActionTest          src/findContactActionTest.cpp)
rosbuild_add_executable(slipServoActionTest          src/slipServoActionTest.cpp)
rosbuild_add_executable(placeActionTest          src/placeActionTest.cpp)
rosbuild_add_executable(forceServoActionTest          src/forceServoActionTest.cpp)


rosbuild_add_executable(pr2_gripper_grab_action          src/pr2_gripper_grab_action.cpp)
rosbuild_add_executable(pr2_gripper_release_action          src/pr2_gripper_release_action.cpp)
rosbuild_add_executable(grabActionTest          src/grabActionTest.cpp)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
