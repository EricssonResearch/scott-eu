#Moveit Config#
This package provides configuration files for moveit!. It was generated using MoveIt! [setup assistant](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) and tailored to work with V-REP ROS controller using as an example the [Interbotix repo](https://github.com/Interbotix/phantomx_pincher_arm).

Package main components are located inside _launch_ and _config_ directory. _Launch_ folder has several files to run Robot's and MoveIt! nodes. The main launch file is demo.launch and it will call all the others _launch_ and config files. Inside _config_ folder, it's nice to pay attention to phantomXPincher\_moveit.yaml that contains the controller's configuration.

##MoveIt! Concepts##
In general, MoveIt! uses robot's URDF to know about robot's configuration, sensors, and actuators. With this information, MoveIt! is capable of planning trajectories, avoid collisions (within the robot and the environment), solve inverse kinematics and so on. To interact with MoveIt! there are interfaces available in C++, Python and rviz provide a GUI interface.

More details can be found at [MoveIt! concepts](http://moveit.ros.org/documentation/concepts/).


##MoveIt! Launch Files##

A detailed description about launch files could be helpful on understanding how MoveIt! components are connected to robot's components.


##How to run##

To work properly this package need the following packages: phantomx\_pincher\_arm\_vrep\_controller and phantomx\_pincher\_arm\_description.

1. Follow the instructions of phantomx\_pincher\_arm\_vrep\_controller README.md and check if it's working
1. Source packages files:
   - on the packages inside phantomx\_pincher\_arm do for each one: `$ source devel/setup.bash`
1. Start roscore
   - on a terminal:
     `$ roscore`
1. Start vrep and load the scene on vrep/turttlebot2\_v4.ttt inside phantomx\_pincher\_arm package
   - It's possible to start vrep in headless mode with.
     - Inside V-REP root folder run:
       `$ ./vrep.sh -h -s -q <PATH_TO_phantomx_pincher_arm_vrep_controller>/vrep/turttlebot2\_v4.ttt`
1. On a new terminal start demo.launch
   - on a terminal:
     	`$ roslaunch phantomx_pincher_arm_moveit demo.launch`
1. If it's everything ok, after while a rviz window will open with a 3D visualization of the arm
1. Control the arm using rviz
   - In the tab _path planning_ inside the moveit! panel it's possible to plan and move the arm to a new position.