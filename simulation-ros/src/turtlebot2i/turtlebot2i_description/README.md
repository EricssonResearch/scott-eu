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
./vrep.sh -s /path/to/turtlebot2i_description/v_rep_model/warehouse_turtlebot2i_v3.ttt
```

**Note:** Plese check if there is a newer version of the warehouse_turtlebot2i_v3.ttt scene file.

2. Execute the roslaunch associated to the robot description

* For a single robot:
```
roslaunch turtlebot2i_description turtlebot2i_description_single.launch

```
**Note:** Enyu checked the launch file name and edit a few lines. 

**Note:** Enyu: it is hard to find the functionality of ROS launch. Because this file just initialize the robot. 

* For multiple robots:
```
roslaunch turtlebot2i_description turtlebot2i_description_multiple.launch
```

* If want to visualize the models and the topic outputs in the rviz:
```
roslaunch turtlebot2i_description turtlebot2i_description_single_rviz.launch
```
or
```
roslaunch turtlebot2i_description turtlebot2i_description_multiple_rviz.launch
```

**Note:** The turtlebot2i_description.launch is the base script used by other scripts to launch the kobuki and the phanthon pincher arm models.

# 3. Loading the Turtlebot2i model and TF using real robots 

To run the description for **one ore two robots**, execute the following steps:

1. Execute the ros launch associated to the robot description

```
roslaunch turtlebot2i_description turtlebot2i_description_multiple_robots.launch
(Enyu didn't edit this part. But this might use a wrong file name.)
```


# 4. Customizing the launch script when using more than two robots

The *turtlebot2i_description_multiple_robots.launch* is limited to be used with up to two robots. In case more robots are used, this script should be modified.

1. Add the following tag for each additional robot, but specify the attribute **robot_name** for the new robot. Ideally, if an additional turtlebot2i is added, keep the 'turtlebot2i_ROBOTID' format, which ROBOTID corresponds to the a integer number.

```xml
  <include file="$(find turtlebot2i_description)/launch/turtlebot2i_description.launch">
    <arg name="robot_name" value="turtlebot2i"/>
  </include>
```

