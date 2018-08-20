#!/usr/bin/env python
import rospy
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseGoal
from actionlib_msgs.msg import GoalID
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped#,Pose,Point,Quaternion
import std_msgs.msg

def quaternion_from_euler_conventor()
    

def talker(): 

    pub = rospy.Publisher('/turtlebot2i/move_base/goal',MoveBaseActionGoal, queue_size=10) 

    rospy.init_node('navi_goal_talker', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
        
    goal_message=MoveBaseActionGoal()    
    goal_message.header=std_msgs.msg.Header()
    goal_message.goal_id=GoalID() #empty
    goal_message.goal=MoveBaseGoal()
    goal_message.goal.target_pose=PoseStamped()
    goal_message.goal.target_pose.header = std_msgs.msg.Header()
    goal_message.goal.target_pose.pose=geometry_msgs.msg.Pose()

    goal_message.header.stamp = rospy.Time.now()

    goal_message.goal.target_pose.header.stamp = rospy.Time.now()
    goal_message.goal.target_pose.header.frame_id="map"#"map"
    ''' Position information
    '''
    goal_message.goal.target_pose.pose.position.x=4.8#-0.8  #4.8
    goal_message.goal.target_pose.pose.position.y=-4.0 #0.5  
    goal_message.goal.target_pose.pose.position.z=0.0
    ''' Orientation info.
    This part now from function quaternion_from_euler_conventor()
    goal_message.goal.target_pose.pose.orientation.x=0.0
    goal_message.goal.target_pose.pose.orientation.y=0.0
    goal_message.goal.target_pose.pose.orientation.z=1.0
    goal_message.goal.target_pose.pose.orientation.w=0.0
    '''
    #'''#this works! But why...
    while not rospy.is_shutdown():
        rospy.loginfo("Publish a navi goal")
        pub.publish(goal_message)
        rate.sleep()
    #'''

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
