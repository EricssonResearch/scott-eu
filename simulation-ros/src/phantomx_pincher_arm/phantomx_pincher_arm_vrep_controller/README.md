controller\_remoteApi - TurtleBot2i
======================================

This package contains a ros\_control backend for the PhantomXArm (TurtleBot2i Arm) simulated on VREP. This package was heavily based on the vrep\_ros\_control\_example available [Here](https://github.com/ros-controls/ros\_control)

### MOTIVATION ###

   The MoveIt! package requires the robot to have an action server to consume trajectory's messages. This kind of server aims to be used when long tasks are being executed and a feedback or a stop action is need while the process is running (http://wiki.ros.org/actionlib). Instead of developing a specific action server to consume MoveIt! messages, the MoveIt documentations advise the usage of a ros_controller (http://moveit.ros.org/documentation/concepts/) as can be seen in the picture below.

   ![alt_text](doc/imgs/moveit.jpg)

   Also, as our project will use a real TurtleBot, it's a good practice to have simulation and real robot operating similarly as possible. 

### ROS_CONTROL ###

   **Disclaimer:** The Ros Control documentation is very sparse and sometimes it's hard to put all the different concepts together. On the following paragraphs, I will give my understanding of how the ROS\_CONTROL works, although it's important to keep in mind that I'm probably wrong or lacking some important concepts. For a more accurate documentation consult the links on the following sections

   The ROS\_CONTROL takes a set-point and joints states. With a lets say, a PID controller, it sends a command to the joints. But to do so, many components must be available.
   Let's consider that we a have a moveIt! node publishing (via actionLib) a trajectory. Thus, we'll a need a controller to read joints states, the input/set point from moveIt! and send commands to joints. To accomplish that, we'll need 3 main components: a Controller Manager, a Controller(Trajectory Controller) and a Hardware Interface.

   * [Controller](http://wiki.ros.org/ros_controllers)
     This can be a simple Controller, like a PID. There are many kinds available on ROS, e.g: position, velocity, effort, trajectory.

   * [Controller Manager](http://wiki.ros.org/controller_manager)
     This component is responsible for spawning a Controller and will do the interface between the Controller and Hardware Interface.
     The Controller Manager will provide an interface to load, start, stop and unload a controller. It will also keep an internal state machine relative to the control loop execution.
     Apparently, the controller manager will manage the interface to read from the Hardware the joints states and write a command to the joints. Inside the controller manager, those interfaces are agnostic from the robot's hardware perspective. So, the Controller Manager provides a general propose interface between the controller and the robot's hardware.

     TLDR.: Provides a general interface between hardware and controller, keep track of the Controller (start, stop, load, unload).

   * [Hardware Interface](https://github.com/ros-controls/ros_control/wiki/hardware_interface)
     
     This component will provide a software abstraction of the robot's hardware. It will make available the respective interfaces to the controller read joint states and send the correct kind of commands to joints. There are three main interfaces joint\_state, velocity, position, and effort.

     **TLDR.:** It's here where you code robot's read and write functions. This will provide an interface to the ros_manager to read states and send commands.

     A diagram to help:

     ![alt text](doc/imgs/how_works.png)

### HOW TO RUN ###
   Based on the original vrep\_ros\_control\_example

   * download the ros packages ros_control and ros_controllers
     - sudo apt-get install ros-kinetic-ros-control
     - sudo apt-get install ros-kinetic-ros-controllers
   * clone this repository in your <workspace>/src folder
   * build the newly cloned package.
     * E.g.: on <workspace> folder run: catkin\_make
   * start roscore
      - on terminal: roscore
   * then start vrep and load vrep/turttlebot2_v4.ttt
   * start the simulation in vrep
   * start another terminal and launch file vrep.launch
      - roslaunch phantomx\_pincher\_arm_vrep\_controller vrep\_phantom.launch
      - it's important to observe the parameters:
      	- vrep_ip: it's the vrep machine's ip
	- vrep_port: it's the remote api port to listen to the robot
	- joints: name of vrep joints that will be controlled
   * start yet another terminal and run action/test_move.py
      - it creates a follow\_joint\_trajectory action to give a goal to your controller and move your robot
      - roscd vrep\_ros\_control\_example
      - cd action
      - ./test\_move.py follow\_joint\_trajectory:=phantom\_controller/follow_joint_trajectory
      - The mobile robot arm should go back for 1 second and forth for another second
      - ./test\_move\_gripper.py gripper\_command:=gripper\_controller/gripper_cmd
      - The arm gripper should open

### HOW IT WORKS ###

To create a ros\_control back end for vrep:

   * obviously you need a .ttt vrep file corresponding to your simulated system
      - It's available at vrep/turttlebot2\_v4.ttt in the current repo
   * The file src/ControlLoop/Phantom\_vrepHW.cpp was modified to acomodate the simulated turtlebot2i hardware interface
      - the file vrepControl\_remoteApi.h or .cpp just implement the node that instantiate the PhantomXPincherArm to expose the vrep hardware interfaces.
   * Unfortunately, you need a redundant description of your robot in urdf (so your robot is described both in the .ttt file and in the urdf)
      - this is because the urdf contains information on the joints and transmissions not contained in the vrep .ttt file
      - to counterbalance this point, note that the urdf importer works pretty well. But once your urdf is imported and you modify something in vrep's .ttt file you need to redundantly modify it in the urdf.

### Notes ###

1 - Please look at the code if you want to understand how the node works, it should be self-explaining for people who know well about vrep and ros\_control, and it is well commented for the others.

2 - The urdf joint names are hard coded and this is not a good practice. It would be good solution to read the joints names and types from the parameter server.

3 - This controller interface uses vrep remote api to comunicate.
