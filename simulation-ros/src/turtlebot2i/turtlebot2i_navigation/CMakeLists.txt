cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(turtlebot2i_navigation)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS kobuki_auto_docking)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS kobuki_auto_docking
)

include_directories(include ${catkin_INCLUDE_DIRS})

# define executable using MyMessage1 etc.
add_executable(docking src/auto_docking_node.cpp)
target_link_libraries(docking ${catkin_LIBRARIES})
add_dependencies(docking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} kobuki_auto_docking)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(kobuki_auto_docking src/auto_docking_node.cpp)
#target_link_libraries(example ${PROJECT_NAME})
