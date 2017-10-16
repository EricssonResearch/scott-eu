cmake_minimum_required(VERSION 2.8.3)
project(base_robot)

find_package(catkin REQUIRED COMPONENTS std_msgs 
                                        geometry_msgs
                                        kobuki_msgs
                                        kobuki_random_walker
                                        roscpp)

find_package(Boost REQUIRED COMPONENTS thread signals)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(turtlebot2i_random_walker src/turtlebot2i_random_walker.cpp) 
target_link_libraries(turtlebot2i_random_walker ${catkin_LIBRARIES})
add_dependencies(turtlebot2i_random_walker ${catkin_EXPORTED_TARGETS})

