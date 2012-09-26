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
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# set include for doxygen documentation
include(${PROJECT_SOURCE_DIR}/doc/UseDoxygen.cmake)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC -msse3 -O3")
include_directories(include/active_realtime_segmentation include/active_realtime_segmentation/pyra src)

#find_package(CUDA QUIET)
#if(CUDA_FOUND)
#  message("Found CUDA.")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")
#  set(LIBS ${LIBS} ${CUDA_LIBRARIES})
#  set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)
#  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} 
#    "-D_DEBUG -arch sm_20 -m64 --ptxas-options -v -c --compiler-bindir=/usr/bin/gcc-4.3")
#  CUDA_COMPILE(CUDA_FILES src/cudasegment.cu )
#endif(CUDA_FOUND)

set(TBB_INSTALL_DIR "/usr")
find_package(TBB QUIET)
if(TBB_FOUND)
  message("Found TBB.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_TBB")
  set(LIBS ${LIBS} ${TBB_LIBRARIES})
endif(TBB_FOUND)

set(cc_sources src/pyra/tpimage.cpp src/pyra/tpimageutil.cpp 
  src/largest.cpp src/fgbgsegment.cpp)
rosbuild_add_library(fgbgsegment ${CUDA_FILES} ${cc_sources})

rosbuild_add_executable(do_segment src/dosegment.cpp)
rosbuild_add_executable(do_segment_node src/dosegment_node.cpp)

target_link_libraries(fgbgsegment ${LIBS})
target_link_libraries(do_segment fgbgsegment)
target_link_libraries(do_segment_node fgbgsegment)

# Demo datasets
# rosbuild_download_test_data (http://www.csc.kth.se/~bohg/testImgDisp.tgz demo/testImgDisp.tgz)
# rosbuild_untar_file (demo/testImgDisp.tgz demo/data/ ALL)

