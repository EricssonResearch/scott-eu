cmake_minimum_required(VERSION 3.0.2)
project(ns3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build)
link_directories(include ${catkin_INCLUDE_DIRS} $ENV{NS3_ROOT}/build/lib)

add_executable(${PROJECT_NAME}_listener src/listener.cpp)
add_executable(${PROJECT_NAME}_talker src/talker.cpp)

set_target_properties(${PROJECT_NAME}_listener PROPERTIES OUTPUT_NAME listener PREFIX "")
set_target_properties(${PROJECT_NAME}_talker PROPERTIES OUTPUT_NAME talker PREFIX "")

target_link_libraries(${PROJECT_NAME}_listener ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_talker ${catkin_LIBRARIES})
