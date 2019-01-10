# 1. Overview

This package provides methods and resources related to the robot safety.
# 2. Scene graph
For scene graph generation, there are two methods in scene_graph folder now. The first one is got it from remote API in Vrep simulator. The second one is got from sensors, such as camera and lidar in Vrep simulator.

See more details in `README.md` file in the folder `./scene_graph`.

# 3. warehouse scenerios 
There are three scenerios now. 
- only robot (change safety zone and reduce speed);
- two robots (change direction);
- one robot and one human (stop when reach red zone);
  
# 4. Safety Message

A custom message *safety_zone_msg* that informs the size of safety zones is provided. It formed by four fields:

- **Header:** Standard ROS header message.
- **clear_safety_zone_radius:** Radius size of the robot's most external safety zone.
- **warning_safety_zone_radius:** Radius size of the robot's intermediary safety zone.
- **critical_safety_zone_radius:** Radius size of the robot's most internal safety zone.
