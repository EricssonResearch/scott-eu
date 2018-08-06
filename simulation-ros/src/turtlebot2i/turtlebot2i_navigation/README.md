# 1. Overview

This package provides methods and resources related to the robot autonomous and non-autonomous navigation.

# 2. Autonomous Navigation

The autonomous navigation is performed through the ROS *move_base* package. This package calculates the local and global path planning and then returns velocity commands (translation and rotation) to the robot. In order to perform the path planning, the following information are required:

- **Robot localization:** Can be provided by ROS AMCL package. Through navigation plugin of Rviz it is possible to estimate the robot initial pose.
- **Robot odometry:** Provided by the robot encoder.
- **Environment map:** Occupancy grid map to inform the occupied and non-occupied areas. This map is converted to the ROS *cost_map* and then used to compute the global path planning.
- **Robot sensors:** Perception data to detect static and dynamic obstacles that is close to the robot. This data is used to construct a local map of the robot surrounding and then used to compute the local path planning.

In the figure below is represented the pipeline of the robot navigation (extracted from [ROS navigation](http://wiki.ros.org/move_base)):

![alt text](http://wiki.ros.org/move_base?action=AttachFile&do=view&target=overview_tf.png "ROS move_base")

## 2.1 Running Autonomous Navigation

The *move_base* can be executed by runnnin the following command:
```
roslaunch turtlebot2i_navigation move_base
```

## 3. Non-Autonomous Navigation

The non-autonomous navigation is performed through manually inputing the velocity commands to the robot. In this sense, the user uses the keyboard to input the robot translation and rotation velocities. This navigation uses the *kobuki_keyop* package provided by Kobuki stack of ROS.

## 3.1 Running Non-Autonomous Navigation

The *move_base* can be executed by runnnin the following command:
```
roslaunch turtlebot2i_navigation turtlebot2i_keyop.launch 
```
