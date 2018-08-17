#!/usr/bin/env python
import rospy
from nav_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
# Should I import the child message file?
def talker(): 
    pub = rospy.Publisher('/turtlebot2i/move_base_simple/goal', MoveBaseGoal, queue_size=10)
    rospy.init_node('navi_goal_talker', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    rospy.loginfo("Publish a navi goal")
    
    goal_message = MoveBaseGoal()
    goal_message.target_pose=PoseStamped()
    goal_message.target_pose.header = std_msgs.msg.Header()
    
    goal_message.target_pose.header

    


while not rospy.is_shutdown():
        rospy.loginfo("Publish a navi goal")
        sg_message=SceneGraph()
        sg_message.header = std_msgs.msg.Header()
        sg_message.header.stamp = rospy.Time.now()
 
        sg_message.object_distance = 5.0
        sg_message.object_direction = 0
        sg_message.object_speed  = 0.0
        sg_message.object_orientation = 0
        pub.publish(sg_message)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
