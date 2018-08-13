
# Instructions for how to start 

1) Add the following to .bashrc

echo "export PYTHONPATH=$PYTHONPATH:$PWD/lib" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/lib" >> ~/.bashrc

This step only needs to be done once.

2) Add the following to .bashrc

source ~/catkin_ws/devel/setup.bash

This step only needs to be done once.


3) Run roscore

 roscore

This is always done first

4)




