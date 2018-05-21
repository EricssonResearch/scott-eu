#!/usr/bin/env python
import time
import roslib;
import rospy
import actionlib
from control_msgs.msg import *
from std_msgs.msg import Float64

JOINT_NAME = ['PhantomXPincher_gripperClose_joint']

client = None

def move1():
    g = GripperCommandGoal()
    g.command = GripperCommand()
    g.command.position = 0.15
    print JOINT_NAME
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
        
        client = actionlib.SimpleActionClient('gripper_command', GripperCommandAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        
        move1()
        
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
