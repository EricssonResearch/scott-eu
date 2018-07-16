
# Instructions for how to start 

1) Install soar library by following the guidelines in this link: https://github.com/KRaizer/Soar-Python-Minimum-Working-Example

2) Add the following to ~/.bashrc

echo "export PYTHONPATH=$PYTHONPATH:$PWD/lib" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/lib" >> ~/.bashrc

This step only needs to be done once.

3) Add the following to ~/.bashrc

source ~/catkin_ws/devel/setup.bash

This step only needs to be done once.

4) Compile the ROS workspace

 cd /PATH/TO/ROS/WS/src
 catkin_make

5) Run roscore

 roscore

This is always done first

6) Run the soar script

 rosrun turtlebot2i_soar soar_robot.py


