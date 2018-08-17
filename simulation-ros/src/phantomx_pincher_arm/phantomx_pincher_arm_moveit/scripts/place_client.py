#! /user/bin/env python

import rospy
import actionlib
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from moveit_commander import roscpp_initialize, RobotCommander


def place_client():
    client = actionlib.SimpleActionClient('place_custom',
                                          moveit_msgs.msg.PlaceAction)

    client.wait_for_server()

    goal = moveit_msgs.msg.PlaceActionGoal().goal
    goal.place_locations.append(moveit_msgs.msg.PlaceLocation())
    goal.place_locations[0].place_pose = geometry_msgs.msg.PoseStamped()
    goal.place_locations[0].place_pose.pose.position.x = 0.17
    goal.place_locations[0].place_pose.pose.position.y = 0.05
    goal.place_locations[0].place_pose.pose.position.z = 0.019
    print goal
    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        roscpp_initialize([])
        rospy.init_node('place_action_client')
        result = place_client()
        print result
    except rospy.ROSInterruptException:
        print('Progam interrupted')
