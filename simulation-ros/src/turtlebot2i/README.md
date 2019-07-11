# 1. Overview

The **turtlebot2i** meta-package (stack) provides the packages related to the Turtlebot2i robot. This meta-package is comprised by the folowing packages:

- **turtlebot2i_description:** Contains the robot description related methods and resources. Through this package it is possible to visualize the robot model and robot TF.
- **turtlebot2i_mapping:** Contains mapping related methods. There are two mapping methods. One using ROS gmapping and other that extracts the obstacles from the V-REP scene.
- **turtlebot2i_navigation:** Contains navigation related methods. This package provides the local path planning through ROS move_base.
- **turtlebot2i_moveit:** Contains motion planning (moveit) related methods. This package integrates with the phantonx robotic arm motion planning with the turtlebot2i.
- **turtlebot2i_warehouse:** Contains all specific methods related to the warehouse.
