# 1. Overview

This package provides methods and resources related to the robot autonomous and non-autonomous navigation.

# 2. Autonomous Navigation

The autonomous navigation is performed through the ROS *move_base* package. This package calculates the local and global path planning and then returns velocity commands (translation and rotation) to the robot. In order to perform the path planning, the following information are required:

- **Robot localization:** Can be provided by ROS AMCL package. Through navigation plugin of Rviz it is possible to estimate the robot initial pose.
- **Robot odometry:** Provided by the robot encoder.
- **Environment map:** Occupancy grid map to inform the occupied and non-occupied areas. This map is converted to the ROS *cost_map* and then used to compute the global path planning.
- **Robot sensors:** Perception data to detect static and dynamic obstacles that is close to the robot. This data is used to construct a local map of the robot surrounding and then used to compute the local path planning.

In the figure below is represented the pipeline of the robot navigation (extracted from [ROS navigation](http://wiki.ros.org/move_base)):

![alt text](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf.png "ROS move_base")

## 2.1 Running Autonomous Navigation

The *move_base* can be executed by running the following command:
```
roslaunch turtlebot2i_navigation move_base.launch
```

The parameters of the navigation algorithm can be changed by editing the configuration in the __config__ folder.
The complete list of the parameters and their description can be checked in the following link: http://wiki.ros.org/navigation/Tutorials/RobotSetup#Navigation_Stack_Setup

After running the launch script, the [rviz](http://wiki.ros.org/navigation/Tutorials/RobotSetup#Navigation_Stack_Setup) will open, which will enable to set iterative parameters of the navigation, such as the robot initial position and the desired destination. The rviz will open with a predefined configuration that connects to the simulated turtlebot2i topics.

To set the robot destination, click on "2D Nav Goal" button and then click on the map to set the goal. The interface will show the trajectory that the robot will follow.
Additional information to use rviz for navigation can be found in this link: http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack

**Note:** The launch file is set up to subscribe predefined topics of a turtlebot2i robot. To change the topic, just edit the move_base.launch file:
```
rosed turtlebot2i_navigation move_base.launch 
```

**Note 2:** A more detailed description of using the navigation stack can be found in this link: http://wiki.ros.org/navigation/Tutorials/RobotSetup

## 3. Non-Autonomous Navigation

The non-autonomous navigation is performed through manually imputing the velocity commands to the robot. In this sense, the user uses the keyboard to input the robot translation and rotation velocities. This navigation uses the *kobuki_keyop* package provided by Kobuki stack of ROS.

## 3.1 Running Non-Autonomous Navigation

The *move_base* can be executed by running the following command:
```
roslaunch turtlebot2i_navigation turtlebot2i_keyop.launch 
```

Use the following commands to move the robot:

1. Once the script is started, enable the robot motors by pressing 'e' (skip this command if using the simulated warehouse scenario).
2. Move robot forward by pressing up key. This increases the linear velocity.
3. Move robot backward by pressing down key. This decreases the linear velocity.
4. Move robot right by pressing right key. This increases the angular velocity.
5. Move robot left by pressing left key. This decreases the angular velocity.


**Note:** The launch file is set up to subscribe predefined topics of a turtlebot2i robot. To change the topic, just edit the turtlebot2i_keyop.launch file:
```
rosed turtlebot2i_navigation turtlebot2i_keyop.launch 
```

## 4. Auto Docking for Recharging


The robot can use the *kobuki_dock_driver* to automatically find the docking station and start recharging the battery itself.
In the simulated environment the recharging station was sligtly adapted and the robot starts to recharge when it is very close to the dock station (< 10 cm).

Use the following steps to start the auto docking process:

1. Start the auto docking actionlib server:

```
roslaunch turtlebot2i_navigation turtlebot2i_auto_docking_server.launch 
```
2. Start the auto docking actionlib client:
```
roslaunch turtlebot2i_navigation turtlebot2i_auto_docking_client.launch 
```

**Note:** Use the option --screen for debugging
