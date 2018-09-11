# This code aims to get some important parameters of robot.
## It should both work for simulator and real robot
## It will send a send a velocity command for 1 sec, so that robot is running with highest speed.
## Then it will send a stop command.
## The robot position is monitored, and the stopping time and stopping distance can be found.

# Run rqt_plot monitor
- Example rqt_plot /turtle1/pose/x:y:z

- For our usecase : rqt_plot /turtlebot2i/odom/pose/pose/position/x:y  # z is not useful.

# Run navi launch file


# Result
 1.5m/s speed. (But from the plot, this is not the correct value. )

From the plot, the speed is about 0.10 m/s.

Stopping time: 8s

Stopping distance: 0.48m
