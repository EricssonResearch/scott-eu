#!/usr/bin/env python
import time
import roslib;
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['front_left_wheel_joint', 'back_left_wheel_joint', 'back_right_wheel_joint', 'front_right_wheel_joint']

Q1_ANGLE = 1.5708
Q0 = [0, 0, 0, 0]
Q1 = [Q1_ANGLE, Q1_ANGLE, Q1_ANGLE, Q1_ANGLE]

client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    print JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q0, time_from_start=rospy.Duration(1.0)),
        JointTrajectoryPoint(positions=Q1, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q0, time_from_start=rospy.Duration(3.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        
        move1()
        
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
