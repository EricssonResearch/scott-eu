cmake_minimum_required(VERSION 2.8.3)
project(turtlebot2i_scene_graph)

list(APPEND MSG_DEPENDS std_msgs sensor_msgs)
list(APPEND CATKIN_DEPENDS rospy ${MSG_DEPENDS})

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS} message_generation)

catkin_python_setup()

add_message_files(FILES SceneGraph.msg)
add_service_files(FILES GenerateSceneGraph.srv)
generate_messages(DEPENDENCIES ${MSG_DEPENDS})

catkin_package(CATKIN_DEPENDS ${CATKIN_DEPENDS} message_runtime)

catkin_install_python(
  PROGRAMS scripts/scene_graph_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/vrep_scene_graph_generator.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
