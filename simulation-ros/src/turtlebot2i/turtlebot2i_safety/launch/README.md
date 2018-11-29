# Main use
 please run **turtlebot2i_safety_single.launch**

It will include the other three files
- **turtlebot2i_navigation_multiple_base**
- **turtlebot2i_keyop**
- **turtlebot2i_cmd_vel_mux**
- **turtlebot2i_risk_single_management.launch** #TODO

please run **turtlebot2i_safety_multiple.launch** for multiple robots

It will include the other five files
- **turtlebot2i_navigation_multiple_base** for turtlebot2i
- **turtlebot2i_keyop**         for turtlebot2i
- **turtlebot2i_cmd_vel_mux**   for turtlebot2i

- **turtlebot2i_navigation_multiple_base** for turtlebot2i#0
- **turtlebot2i_cmd_vel_mux**   for turtlebot2i#0 #TODO

- **turtlebot2i_risk_multiple_management.launch** #TODO
# Note:

Please note that in the **"turtlebot2i_cmd_vel_mux"**, **"nodelet manager"** is also needed!

# Todo:

YAML (vel_mux, in the param folder) file should be changed. This file need the name of topics. But different robots will have diffierent prefixs. **Currently, this YAML can only be used for one robot.**

# For labeling
please run **turtlebot2i_safety_single_labelling.launch**

It will include 
- **turtlebot2i_navigation_single**
- **turtlebot2i_keyop**
- **turtlebot2i_cmd_vel_mux**
- **turtlebot2i_scene_graph_labelling**

# To visualize SceneGraph
please run **turtlebot2i_scene_graph.launch**
