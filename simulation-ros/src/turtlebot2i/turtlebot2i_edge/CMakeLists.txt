cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2i_edge)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

list(
  APPEND MSG_DEPENDS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
  kobuki_msgs
  actionlib_msgs
  turtlebot2i_scene_graph
)
list(APPEND CATKIN_RUN_DEPENDS roscpp rospy actionlib message_filters cv_bridge message_runtime ${MSG_DEPENDS})
list(APPEND CATKIN_BUILD_DEPENDS roscpp rospy actionlib message_filters cv_bridge message_generation ${MSG_DEPENDS})

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPENDS})
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
find_library(
  NS3_BUILDINGS_LIBRARY
  NAMES ns3-dev-buildings-optimized ns3-dev-netanim-debug
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
  ${NS3_BUILDINGS_LIBRARY}
)

catkin_python_setup()

add_message_files(FILES NetworkLatency.msg NetworkThroughput.msg)
add_service_files(FILES GenerateSceneGraph.srv Stamp.srv)
generate_messages(DEPENDENCIES ${MSG_DEPENDS})

catkin_package(CATKIN_DEPENDS ${CATKIN_RUN_DEPENDS})

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build)

add_executable(${PROJECT_NAME}_ns3_wifi_adhoc src/ns3_wifi_adhoc.cpp)
set_target_properties(${PROJECT_NAME}_ns3_wifi_adhoc PROPERTIES OUTPUT_NAME ns3_wifi_adhoc PREFIX "")
target_link_libraries(${PROJECT_NAME}_ns3_wifi_adhoc ${catkin_LIBRARIES} ${Threads_LIBRARIES} ${NS3_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_ns3_wifi_adhoc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/task_offloading.py
  scripts/scene_graph_proxy.py
  scripts/network_latency_monitor.py
  scripts/network_throughput_monitor.py
  scripts/stamper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/activate_netns.sh scripts/netns_wifi_adhoc.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)