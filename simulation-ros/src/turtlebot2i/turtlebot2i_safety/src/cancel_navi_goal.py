#!/usr/bin/env python
#or simply put this on terminal: rostopic pub /turtlebot2i/move_base/cancel actionlib_msgs/GoalID -- {}

import rospy
from actionlib_msgs.msg import GoalID


def talker():

    pub = rospy.Publisher('/turtlebot2i/move_base/cancel',GoalID, queue_size=10)
    rospy.init_node('cancel_navi_goal', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    cancel_message=GoalID() #empty

    i=0
    while i<10:
        rospy.loginfo("Publish a cancellation")
        pub.publish(cancel_message)
        rate.sleep()
        i+=1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
