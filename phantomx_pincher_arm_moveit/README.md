#Moveit Config#

This package provides configuration files for moveit!. The main components are located inside _launch_ and _config_ directory. _Launch_ folder has several files to run Robot's and MoveIt! nodes. The main launch file is demo.launch and it will call all the others _launch_ and config files. Inside _config_ folder it's nice to pay attention to phantomXPincher\_moveit.yaml that contains the controllers configuration.

##MoveIt! Concepts##


##MoveIt! Launch Files##


##How to run##

To work properly this package need the following packages: phantomx\_pincher\_arm\_vrep\_controller and phantomx\_pincher\_arm\_description.

1- Follow the instructions of phantomx_pincher_arm_vrep_controller README.md and check if it's working

2- Source packages files:

   - inside phantomx\_pincher\_arm folder:
     - source devel/setup.bash

3- Start roscore

  - on a terminal: roscore

4- Start vrep and load the scene on vrep/turttlebot2\_v4 inside phantomx\_pincher\_arm package

   1- It's possible to start vrep in headless mode with.
   
      - Inside V-REP root folder run:
      	./vrep.sh -h -s -q PATH\_TO\_phantomx\_pincher\_arm\_vrep\_controller/vrep/turttlebot2\_v4.ttt

5- On a new terminal start demo.launch

  - on a terminal: roslaunch phantomx\_pincher\_arm\_moveit demo.launch

6- If it's everything ok, after while a rviz window will open with a 3D visualization of the arm

7- In the tab _path planning_ within the moveit! panel it's possible to plan and move the arm to a new position.