cmake_minimum_required(VERSION 2.8.3)
project(phantomx_pincher_arm_vrep_controller)

find_package(catkin REQUIRED COMPONENTS roscpp controller_manager)

################################################
# List sources.
################################################
set(${PROJECT_NAME}_headers
    src/v_repConst.h
    src/v_repLib.h
    src/v_repTypes.h
#    src/ControlLoop/vrepRosControl_server.h
#    src/ControlLoop/vrepRosControl_plugin.h
    src/ControlLoop/Phantom_vrepHW.h
    src/ControlLoop/vrepRosControl_remoteApi.h
    src/extApi.h
    src/extApiInternal.h
    src/extApiPlatform.h
)

set(${PROJECT_NAME}_sources
#    src/v_repLib.cpp
#    src/ControlLoop/vrepRosControl_server.cpp
#    src/ControlLoop/vrepRosControl_plugin.cpp
    src/ControlLoop/vrepRosControl_remoteApi.cpp
    src/ControlLoop/Phantom_vrepHW.cpp
    src/extApi.c
    src/extApiPlatform.c
)

catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs roslib
               DEPENDS roscpp rosconsole std_msgs roslib)

include_directories(include ${catkin_INCLUDE_DIRS})

#link_directories("/opt/ros/kinetic/lib")

#add_library(v_repExtRosControl ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

#add_dependencies(v_repExtRosControl ${catkin_EXPORTED_TARGETS})
#target_link_libraries(v_repExtRosControl
#    pthread
#    dl
#    roslib
#    rosconsole
#    rostime
#    rospack
#    roscpp_serialization
#    roscpp
#    boost_system
#   ${catkin_LIBRARIES}
#)

add_executable(controller_remoteApi
  ${${PROJECT_NAME}_sources}
  ${${PROJECT_NAME}_headers}
)
target_link_libraries(controller_remoteApi
  ${catkin_LIBRARIES}
)
target_compile_options(controller_remoteApi
  PRIVATE -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
)

add_dependencies(controller_remoteApi ${catkin_EXPORTED_TARGETS})
