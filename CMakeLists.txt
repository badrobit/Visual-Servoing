cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#..: Create this as a ROS CMake :.............................................#
rosbuild_init()

#..: ROS Dynamic Reconfigure :................................................#
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#..: OpenCV :.................................................................#
find_package( OpenCV )

#..: Default Paths :..........................................................#
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/common/include
	${PROJECT_SOURCE_DIR}/ros/include
)

#..: 2D Visual Servoing Library :.............................................#
rosbuild_add_library( VisualServoing2D common/src/VisualServoing2D.cpp )
target_link_libraries( VisualServoing2D cvblobs 
										${OpenCV_LIBRARIES} )

#..: 3D Visual Servoing Library :.............................................#
#rosbuild_add_library( VisualServoing3D common/src/VisualServoing3D.cpp )

#..: Visual Seroving 2D Node :................................................#
rosbuild_add_executable(visual_servoing_node ros/src/visual_servoing.cpp)
target_link_libraries(visual_servoing_node VisualServoing2D )