cmake_minimum_required(VERSION 2.8.3)
project(base_robot)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
    endif()

find_package(catkin REQUIRED COMPONENTS std_msgs 
                                        geometry_msgs
                                        kobuki_msgs
                                        kobuki_random_walker
                                        kobuki_safety_controller
                                        yocs_cmd_vel_mux
                                        roscpp)

find_package(Boost REQUIRED COMPONENTS thread signals)

#include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(turtlebot2i_random_walker src/turtlebot2i_random_walker.cpp) 
add_executable(turtlebot2i_safety_controller src/turtlebot2i_safety_controller.cpp) 
add_executable(turtlebot2i_cmd_vel_mux src/turtlebot2i_cmd_vel_mux.cpp) 

target_link_libraries(turtlebot2i_random_walker ${catkin_LIBRARIES})
add_dependencies(turtlebot2i_random_walker ${catkin_EXPORTED_TARGETS})

target_link_libraries(turtlebot2i_safety_controller ${catkin_LIBRARIES})
add_dependencies(turtlebot2i_safety_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(turtlebot2i_cmd_vel_mux ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})
add_dependencies(turtlebot2i_cmd_vel_mux ${catkin_EXPORTED_TARGETS})
