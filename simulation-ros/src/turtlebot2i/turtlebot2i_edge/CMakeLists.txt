cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2i_edge)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(Threads REQUIRED)
find_library(ns3_LIBRARIES NAMES
  ns3-dev-core-optimized
  ns3-dev-network-optimized
  ns3-dev-wifi-optimized
  ns3-dev-tap-bridge-optimized
  ns3-dev-mobility-optimized
  HINTS $ENV{NS3_ROOT}/lib $ENV{NS3_ROOT}/lib64
  REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/include/ns3-dev)

add_executable(${PROJECT_NAME}_ros_ns3 src/ros_ns3.cpp)
set_target_properties(${PROJECT_NAME}_ros_ns3 PROPERTIES OUTPUT_NAME ros_ns3 PREFIX "")
target_link_libraries(
  ${PROJECT_NAME}_ros_ns3
  ${catkin_LIBRARIES}
  ${Threads_LIBRARIES}
  ${ns3_LIBRARIES}
)

# TODO
#catkin_install_python(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# TODO
#install(TARGETS ${PROJECT_NAME}_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)