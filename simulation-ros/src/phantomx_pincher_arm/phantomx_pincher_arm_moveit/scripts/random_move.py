#!/usr/bin/env python
import sys
import rospy
import geometry_msgs.msg
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState

if __name__ == '__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    print "Current state:"
    robot.get_current_state()

    a = robot.pincher_arm
    print a.get_joints()

    a.set_num_planning_attempts(10000)
    a.set_planning_time(5)

    pose_now = a.get_current_pose()
    print pose_now

    a.set_start_state(RobotState())

    print "Tolerances"
    a.set_goal_position_tolerance(0.005)
    a.set_goal_orientation_tolerance(1)
    print a.get_goal_position_tolerance()
    print a.get_goal_orientation_tolerance()
    
    x = 0.20
    y = 0.
    z = 0.1
    r = a.set_position_target([x,y,z])
    print "Planning to position: "
    print [x,y,z]

    rospy.sleep(1)

    p = a.plan()
    print "Solution:"
    print p
    a.execute(p)

    rospy.sleep(15)
    print a.get_current_pose()
    roscpp_shutdown()
