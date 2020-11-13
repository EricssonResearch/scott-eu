cmake_minimum_required(VERSION 3.0.2)
project(ns3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package(Threads REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build)
link_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build/lib)

add_executable(${PROJECT_NAME}_listener src/listener.cpp)
add_executable(${PROJECT_NAME}_talker src/talker.cpp)
add_executable(${PROJECT_NAME}_ros_ns3 src/ros_ns3.cpp)

set_target_properties(${PROJECT_NAME}_listener PROPERTIES OUTPUT_NAME listener PREFIX "")
set_target_properties(${PROJECT_NAME}_talker PROPERTIES OUTPUT_NAME talker PREFIX "")
set_target_properties(${PROJECT_NAME}_ros_ns3 PROPERTIES OUTPUT_NAME ros_ns3 PREFIX "")

target_link_libraries(${PROJECT_NAME}_listener ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_talker ${catkin_LIBRARIES})
target_link_libraries(      # TODO: compile not for debugging and use optimized versions
    ${PROJECT_NAME}_ros_ns3
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ns3.32-core-debug
    ns3.32-network-debug
    ns3.32-wifi-debug
    ns3.32-tap-bridge-debug
    ns3.32-mobility-debug
    ns3.32-stats-debug
)