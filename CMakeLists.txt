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

#..: Default Paths :..........................................................#
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#..: Visual Seroving 2D Node :................................................#
rosbuild_add_executable(visual_servoing_node ros/src/visual_servoing.cpp)
target_link_libraries(visual_servoing_node cvblobs)

#..: 3D Visual Servoing Library :.............................................#
rosbuild_add_library( VisualServoing3D common/src/VisualServoing3D.cpp )