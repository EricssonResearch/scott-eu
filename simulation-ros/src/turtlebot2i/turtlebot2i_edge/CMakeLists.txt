cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2i_edge)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(Threads REQUIRED)

find_library(
  NS3_CORE_LIBRARY
  NAMES ns3-dev-core-optimized ns3-dev-core-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_NETWORK_LIBRARY
  NAMES ns3-dev-network-optimized ns3-dev-network-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_WIFI_LIBRARY
  NAMES ns3-dev-wifi-optimized ns3-dev-wifi-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_POINT_TO_POINT_LIBRARY
  NAMES ns3-dev-point-to-point-optimized ns3-dev-point-to-point-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_INTERNET_LIBRARY
  NAMES ns3-dev-internet-optimized ns3-dev-internet-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_TAP_BRIDGE_LIBRARY
  NAMES ns3-dev-tap-bridge-optimized ns3-dev-tap-bridge-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_MOBILITY_LIBRARY
  NAMES ns3-dev-mobility-optimized ns3-dev-mobility-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)
find_library(
  NS3_STATS_LIBRARY
  NAMES ns3-dev-stats-optimized ns3-dev-stats-debug
  HINTS $ENV{NS3_ROOT}/build/lib
  REQUIRED
)

list(
  APPEND NS3_LIBRARIES
  ${NS3_CORE_LIBRARY}
  ${NS3_NETWORK_LIBRARY}
  ${NS3_WIFI_LIBRARY}
  ${NS3_POINT_TO_POINT_LIBRARY}
  ${NS3_INTERNET_LIBRARY}
  ${NS3_TAP_BRIDGE_LIBRARY}
  ${NS3_MOBILITY_LIBRARY}
  ${NS3_STATS_LIBRARY}
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build)

add_executable(${PROJECT_NAME}_ns3_wifi src/ns3_wifi.cpp)
set_target_properties(${PROJECT_NAME}_ns3_wifi PROPERTIES OUTPUT_NAME ns3_wifi PREFIX "")
target_link_libraries(
  ${PROJECT_NAME}_ns3_wifi
  ${catkin_LIBRARIES}
  ${Threads_LIBRARIES}
  ${NS3_LIBRARIES}
)

# TODO
#catkin_install_python(
#  PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

#install(TARGETS ${PROJECT_NAME}_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
