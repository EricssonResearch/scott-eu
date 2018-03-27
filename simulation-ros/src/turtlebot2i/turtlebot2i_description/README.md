# 1. Overview

This package provides the resources to visualize the Turtlebot2i robot and load the [TF](http://wiki.ros.org/tf) structure of the robot. Additionally, the V-REP related resources are sotored in this package, such as the warehouse scene and the lua scripts necessary to run the simulation. The package contains the following folders:

1. urdf: Contains the visualization and transformation models.
2. robots: Contains the main urdf files.
3. meshes: Contains the robots and sensors 3D models.
4. v_rep_model: Contains V-REP scenarios and lua scripts.
5. launch: ROS launch scripts to call the methods to compose the robot model.
6. rviz: Contains rviz saved preconfigured profiles to visualize the robot model.


# 2. Loading the Turtlebot2i model and TF using simulated robots (V-REP)

To run the description for **one ore two robots**, execute the following steps:

1. Run the scene containing the turtlebot2i

```
./vrep.sh -s /path/to/turtlebot2i_description/v_rep_model/turtlebot2i_Warehouse.ttt
```

2. Execute the ros launch associated to the robot description

```
roslaunch turtlebot2i_description turtlebot2i_description_multiple_robots.launch
```


# 3. Loading the Turtlebot2i model and TF using real robots 

To run the description for **one ore two robots**, execute the following steps:

1. Execute the ros launch associated to the robot description

```
roslaunch turtlebot2i_description turtlebot2i_description_multiple_robots.launch
```


# 4. Customizing the launch script when using more than two robots

The *turtlebot2i_description_multiple_robots.launch* is limited to be used with up to two robots. In case more robots are used, this script should be modified.

1. Add the following tag for each additional robot, but specify the attribute **robot_name** for the new robot. Ideally, if an additional turtlebot2i is added, keep the 'turtlebot2i_ROBOTID' format, which ROBOTID corresponds to the a integer number.

```xml
  <include file="$(find turtlebot2i_description)/launch/turtlebot2i_description.launch">
    <arg name="robot_name" value="turtlebot2i"/>
  </include>
```

# 5. Steps to test the function of safety zone size change 
If you just want to test the safety zone function, `catkin build` is necessary in the root path (`/yourpath/scout-eu/simulation-ros`) . Please also do `source devel/setup.bash` in the root path.

1. Run `roscore`.
2. Start V-rep and open the scenaro  `turtlebot2i.ttt` from the path `/yourpath/scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_description` and run it.
3. Send ros topic for a new terminal, the format of this message (change safety zone size) should be like:
```
$ rostopic pub /turtlebot/commands/setSafetySize turtlebot2i_safety/SafetyZone "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
clear_zone_radius: 1.8
warning_zone_radius: 1.3
critical_zone_radius: 1.0" 

```
Please note: **Don't** save the scene after your finish the simulation. Otherwise you need to modify geometry of the safety zone manually to 1 meter in the beginning of next simulation, or the size of the safety zone in the simulator might display incorrectly.

