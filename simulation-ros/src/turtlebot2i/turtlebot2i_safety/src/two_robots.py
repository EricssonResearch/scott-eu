#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
import math
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import SafetyZone
import numpy as np
from kobuki_msgs.msg import BumperEvent

def init_var():
    #Here we initialize the global variables.
    orientation=geometry_msgs.msg.Quaternion()
    orientation=quaternion_from_euler(0,0,0)#(roll, pitch,yaw) # return an array
    
    global goal1, goal2
    goal1 = MoveBaseGoal()
    goal1.target_pose.header.frame_id = "map"
    goal1.target_pose.header.stamp = rospy.Time.now()
    goal1.target_pose.pose.position.x = -2.0
    goal1.target_pose.pose.position.y = -2.5
    goal1.target_pose.pose.position.z = 0.063 #1.34851861
    goal1.target_pose.pose.orientation.x=0.0
    goal1.target_pose.pose.orientation.y=0.0
    goal1.target_pose.pose.orientation.z=orientation[2]
    goal1.target_pose.pose.orientation.w=orientation[3]


    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map"
    goal2.target_pose.header.stamp = rospy.Time.now()
    goal2.target_pose.pose.position.x = -4.5
    goal2.target_pose.pose.position.y =  3.0
    goal2.target_pose.pose.position.z = 0.063 #1.34851861
    goal2.target_pose.pose.orientation.x=0.0
    goal2.target_pose.pose.orientation.y=0.0
    goal2.target_pose.pose.orientation.z=orientation[2]
    goal2.target_pose.pose.orientation.w=orientation[3]
    
    global client1, client2
    client1 = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client2 = actionlib.SimpleActionClient('turtlebot2i_0/move_base', MoveBaseAction)

    global rob1goal, rob2goal
    rob1goal = True
    rob2goal = True

    global time_start, prev_pos, init_pos, pose_cb_count, travelled_distance, sum_distance_to_goal 
    time_start = rospy.get_time()
    init_pos = geometry_msgs.msg.Point()
    prev_pos = geometry_msgs.msg.Point()
    curr_pos = geometry_msgs.msg.Point()
    pose_cb_count        = 0
    travelled_distance   = 0.0 #the less the better
    sum_distance_to_goal = 0.0 #the less the better
    

def move_to_goal(goal, client):
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i_0/move_base', MoveBaseAction)
    
    client.wait_for_server()
    client.send_goal(goal)
    print("Goal position is sent! waiting the robot to finish....")
    wait = client.wait_for_result(timeout=rospy.Duration(1200.0)) #timeout in seconds
    if not wait:
        rospy.logerr("Action server not available or timeout!")
        rospy.signal_shutdown("Action server not available!")

def distance2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

def init_subscription():
    time_start = rospy.get_time()
    rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback1)
    rospy.Subscriber('/turtlebot2i_0/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback2)
 
def update_pose_callback1(data):
    global rob1goal
    if rob1goal:
        if distance2D(data.pose.position, goal1.target_pose.pose.position) < 0.2: #check distance to goal
            print("goal reached!a")
            client1.cancel_all_goals()
            client1.send_goal(goal2)
            rob1goal = False
    else:
        if distance2D(data.pose.position, goal2.target_pose.pose.position) < 0.2: #check distance to goal
            print("goal reached!b")
            client1.cancel_all_goals()
            client1.send_goal(goal1)
            rob1goal = True

def update_pose_callback2(data):
    global rob2goal
    if rob2goal:
        if distance2D(data.pose.position, goal2.target_pose.pose.position) < 0.2: #check distance to goal
            print("goal reached!c")
            client2.cancel_all_goals()
            client2.send_goal(goal1)
            rob2goal = False
    else:
        if distance2D(data.pose.position, goal1.target_pose.pose.position) < 0.2: #check distance to goal
            print("goal reached!d")
            client2.cancel_all_goals()
            client2.send_goal(goal2)
            rob2goal = True

 

if __name__ == '__main__':
    try:
        rospy.init_node('test_run_py')
        init_var()
        init_subscription()
        client1.wait_for_server()
        client2.wait_for_server()
        print("Both clients are ready, sending the goals")
        client1.send_goal(goal1)
        client2.send_goal(goal2)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
