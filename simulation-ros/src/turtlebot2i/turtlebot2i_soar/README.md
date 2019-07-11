
# Instructions for how to start 

0) Install simulation-ros system as described in this link: https://github.com/EricssonResearch/scott-eu/blob/simulation-ros/simulation-ros/doc/README.md

1) Install soar library by following the guidelines in this link: https://github.com/KRaizer/Soar-Python-Minimum-Working-Example

2) Add the following to ~/.bashrc by running on terminal:

```
$ echo "export PYTHONPATH=$PYTHONPATH:$PWD/lib" >> ~/.bashrc
$ echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/lib" >> ~/.bashrc
```

This step only needs to be done once.

3) Add the following to ~/.bashrc

```
$ source <path_to_repository>/scott-eu/simulation-ros/devel/setup.bash
```

This step only needs to be done once.

4) Compile the ROS workspace

```
$ cd /PATH/TO/ROS/WS/
$ catkin_make
```

Example:
```
$ cd <path_to_repository>/scott-eu/simulation-ros
$ catkin_make
```

If an error appears complaining about the presence of build, run the following instead:

```
$ catkin build
```

5) Run roscore
```
$ roscore
```

This is always done first

6) Run the soar script
```
$ rosrun turtlebot2i_soar soar_robot.py
```

