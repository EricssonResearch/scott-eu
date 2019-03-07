#!/usr/bin/env python

"""
    Edited from navigation.py in turtlebot2i_navigation module
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
#from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

def init_var():
    '''
    Here we initialize the global variables.
    '''
    global time_start, prev_pos, init_pos, travelled_distance, decimal_places
    time_start = rospy.get_time()
    init_pos = geometry_msgs.msg.Point()
    prev_pos = geometry_msgs.msg.Point()
    curr_pos = geometry_msgs.msg.Point()
    travelled_distance = 0.0
    decimal_places = 6

def movebase_client():
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #client = actionlib.SimpleActionClient('turtlebot2i_0/move_base', MoveBaseAction)
    client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.2230 #3
    goal.target_pose.pose.position.y = -3.96485 #3
    goal.target_pose.pose.position.z = 0.063 #1.34851861
    #goal.target_pose.pose.orientation.w = 1.0

    #copied from navi_goal_talker
    orientation=geometry_msgs.msg.Quaternion()
    yaw  =-90*math.pi/180 #unit: from deg. to rad.
    orientation=quaternion_from_euler(0,0,yaw)#(roll, pitch,yaw) # return an array
    goal.target_pose.pose.orientation.x=0.0
    goal.target_pose.pose.orientation.y=0.0
    goal.target_pose.pose.orientation.z=orientation[2]
    goal.target_pose.pose.orientation.w=orientation[3]

    client.send_goal(goal, active_cb=init_subscription, done_cb=finish_callback)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
        #return client.get_result()

def finish_callback(state_data, data2):
    time_finish = rospy.get_time()
    duration = time_finish - time_start
    rospy.loginfo("finished in: %1.4f seconds, with distance=%1.4f",duration,travelled_distance)

def init_subscription():
    time_start = rospy.get_time()
    rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, update_pose_callback)

def update_pose_callback(data):
    global prev_pos, init_pos, travelled_distance, decimal_places
    if prev_pos == init_pos:
        prev_pos = data.pose.position
    else:
        curr_pos = data.pose.position
        travelled_distance += math.sqrt((curr_pos.x - prev_pos.x)**2 + (curr_pos.y - prev_pos.y)**2)
        prev_pos = data.pose.position

if __name__ == '__main__':
    try:
        rospy.init_node('test_run_py')
        init_var()
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
