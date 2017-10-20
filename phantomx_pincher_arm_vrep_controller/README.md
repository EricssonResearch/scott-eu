vrep_ros_control_example - TurtleBot2i
======================================

This package contains a ros_control back end for the PhantomXArm (TurtleBot2i Arm) simulated on VREP. This package was based on the vrep_ros_control_example available at https://github.com/ros-controls/ros_control

MOTIVATION

   Packages that are broadly used like the MoveIt! require the robot to have an action server to consume trajectory's messages. This kind of server aim to be used when an long tasks is being executed and a feedback or a stop action is need while the process is running (http://wiki.ros.org/actionlib). Instead of developing a specific action server to consume MoveIt! messages, the MoveIt documentations advise to use a ros_controller (http://moveit.ros.org/documentation/concepts/) as can be seen on the picture below.

   Also, as our project will use a real TurtleBot, is a good practice to have simulation and real robot operating as close as possible. And the TurtleBot uses the ros_controller packages to manage the arm movements.

ROS_CONTROL

   Disclaimer: The documentation is very sparse and sometimes it's hard to put all the different concepts together. On the following paragraphs I will give my understand about how the ROS_CONTROL works, although it's important to keep in mind that I'm probably wrong.

   The ROS_CONTROL basically takes an set_point and the joints states. With a, lets say a PID controller, it sends a command to the joints. But, in order to do so many components must be available.
   Lets consider that we a have a moveIt! node publishing (via actionLib) a trajectory, so we a need a controller to read joints states and the input from moveIt! and send commands to joints. To accomplish that, we'll need 3 main components: a Controller Manager, a Controller(Trajectory Controller) and a Hardware Interface.

   * Controller
     Briefly: This can be a simple Controller, like a PID. There's many kinds available on ROS, E.g: position, velocity, effort, trajectory.
     http://wiki.ros.org/ros_controllers

   * Controller Manager
     http://wiki.ros.org/controller_manager
     This component is responsible to spawn a Controller and will do the interface between the Controller and Hardware Interface.
     The Controller Manager will provide an interface to load, start, stop and unload a controller. It will also keep an internal state machine relative to the control loop execution.
     Apparently, the controller manager will manage the interface to read from the Hardware the joints states and write a command to the joints. Inside the controller manager, those interfaces are agnostic from the robot's hardware perspective. So, the Controller Manager provides a general propose interface between the controller and the robot's hardware.

     TLDR.: Provides an general interface between hardware and controller, keep track of the Controller (start, stop, load, unload).
   * Hardware Interface
     https://github.com/ros-controls/ros_control/wiki/hardware_interface
     
     This component will provide an software abstraction of the robot's hardware. It will made available the respective interfaces to the controller read joint states and send the correct kind of commands to joints. There are three main interfaces joint_state, velocity, position and effort.

     TLDR.: It's here where you code read and write functions. This will provide an interface to the ros_manager to read states and send commands.

     A diagram to help:

     ![alt text](how_works.png)

HOW TO RUN IT (based on the original vrep_ros_control_example)

   * clone the repository in your <workspace>/src folder
   * build the newly cloned package.
     * E.g.: on <workspace>/src folder run: catkin_make
   * once built, you need to copy this plugin file libv_repExtRosControl.so from your <workspace>/build/lib folder to vrep's main folder (along all the other plugins)
   * start a terminal with roscore
   * then start vrep and load vrep/turttlebot2_v4.ttt
      - check that plugin libv_repExtRosControl.so is correctly loaded in vrep's trace (i.e. in vrep's console)
   * start the simulation in vrep
   * start another terminal and launch file vrep.launch
      - roslaunch vrep_ros_control_example vrep_phantom.launch
   * start yet another terminal and run action/test_move.py
      - it creates a follow_joint_trajectory action so as to give a goal to your controller and move your robot
      - roscd vrep_ros_control_example
      - cd action
      - ./test_move.py follow_joint_trajectory:=phantom_controller/follow_joint_trajectory
      - The mobile robot arm should go back for 1 second and forth for another second
      - ./test_move_gripper.py gripper_command:=gripper_controller/gripper_cmd
      - The arm gripper should open

HOW IT WORKS

In order to create a ros_control back end for vrep:
   * obviously you need a .ttt vrep file corresponding to your simulated system
      - this is file vrep/turttlebot2_v4.ttt in the current case
   * The file src/ControlLoop/MyRobot_vrepHW.cpp was modified to meet the simulated turtlebot2i hardware interface
      - the other files like vrepControl_plugin .h or .cpp and vrepControl_server .h or .cpp are generic and should not be changed
      - files src/v_rep* are directly copied from vrep (and needed to create a plugin), they should not be changed
   * unfortunately you need a redundant description of your robot in urdf (so your robot is described both in the .ttt file and in the urdf)
      - this is because the urdf contains information on the joints and transmissions not contained in the vrep .ttt file
      - to counterbalance this point, note that the urdf importer works pretty well. But once your urdf is imported and you modify something in vrep's .ttt file you need to redundantly modify it in the urdf (I told Coppelia about this situation but unfortunately did not receive an answer)
   * this plugin is based on the blank ros skeleton plugin vrep_plugin_skeleton provided in vrep's folder programming/ros_packages/vrep_plugin_skeleton

Please look at the code if you want to understand how the plugin works, it should be self explaining for people who know well about vrep and ros_control, and it is well commented for the others. The main "trick" is in function "bool ROS_server::initialize()" in file vrepControl_server.cpp.

In case of trouble feel free to contact me.

Thanks,

Amadeu do Nascimento Jr.

amadeu.nascimento.junior@ericsson.com
