# 1. Overview

This package provides methods and resources related to the robot safety.

# 2. Scripts

This package provides several scripts with different functionalities.

- **navi_goal_talker** (Developing) is used to manually publish a navigation goal to move_base.

- **safety_assessment_simple_talker** (not useful now, will be removed soon) is used to send a scene graph message. (Note: the SG message has been changed. use SceneGraphV1.msg if you want to test it)

- **safety_assessment_one_object** (not useful now, will be removed soon) is a demo of FLS.

- **GenerateSceneGraph** (Developed) folder contains neccessary files to get scene graph from Vrep remote API. Run **sg_generator.py** and monitor '/turtlebot2i_safety/SceneGraph' topic

- **parse_sg.py** (Developing) is used to parse SG. It is under development

# 3. Message

A custom message *safety_zone_msg* that informs the size of safety zones is provided. It formed by four fields:

- **Header:** Standard ROS header message.
- **clear_safety_zone_radius:** Radius size of the robot's most external safety zone.
- **warning_safety_zone_radius:** Radius size of the robot's intermediary safety zone.
- **critical_safety_zone_radius:** Radius size of the robot's most internal safety zone.

*SceneGraph.msg* contains a string in DOT format.

*VelocityScale.msg* (not decided whether use this topic or not )
