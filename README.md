# Repository for V-REP model of Turtlebot2i

## Pre-requisites

0. Install V-REP and ROS
   V-REP: http://www.coppeliarobotics.com/files/tmp
   ROS:   http://wiki.ros.org/kinetic/Installation

1. Install Turttlebot2i packages from ROS
  sudo apt install ros-kinetic-turtlebot* libudev-dev ros-kinetic-find-object-2d ros-kinetic-rtabmap-ros
  ros-kinetic-moveit ros-kinetic-octomap-ros ros-kinetic-manipulation-msgs ros-kinetic-controller-manager python-wxgtk3.0

2. Install V-REP ROS Interface
  Follow the instructions inside $VREP_ROOT/programming/ros_packages/ros_vrep_rosinterface_install_guide.txt

3. Enable ROS Turttlebot2i messages for V-REP
    1. Move to vrep_ros_interface folder inside ROS workspace
      roscd vrep_ros_interface
    2. Change CMakeLists.txt
      Add inside catkin_package() list: actionlib_msgs and kobuki_msgs
    3. Change package.xml
      Add inside <package></package> tag, the following: 
        <run_depend>actionlib_msgs</run_depend>
        <run_depend>kobuki_msgs</run_depend>
    4. Change meta/messages.txt
      Add the following lines:
        actionlib_msgs/GoalID
        actionlib_msgs/GoalStatusArray
        actionlib_msgs/GoalStatus
        kobuki_msgs/AutoDockingActionFeedback
        kobuki_msgs/AutoDockingActionGoal
        kobuki_msgs/AutoDockingAction
        kobuki_msgs/AutoDockingActionResult
        kobuki_msgs/AutoDockingFeedback
        kobuki_msgs/AutoDockingGoal
        kobuki_msgs/AutoDockingResult
        kobuki_msgs/BumperEvent
        kobuki_msgs/ButtonEvent
        kobuki_msgs/CliffEvent
        kobuki_msgs/ControllerInfo
        kobuki_msgs/DigitalInputEvent
        kobuki_msgs/DigitalOutput
        kobuki_msgs/DockInfraRed
        kobuki_msgs/ExternalPower
        kobuki_msgs/KeyboardInput
        kobuki_msgs/Led
        kobuki_msgs/MotorPower
        kobuki_msgs/PowerSystemEvent
        kobuki_msgs/RobotStateEvent
        kobuki_msgs/ScanAngle
        kobuki_msgs/SensorState
        kobuki_msgs/Sound
        kobuki_msgs/VersionInfo
        kobuki_msgs/WheelDropEvent
    5. Recompile vrep_ros_package
    6. Install the updated vrep_ros_interface library in the V-REP
      rosrun vrep_ros_interface install.sh
