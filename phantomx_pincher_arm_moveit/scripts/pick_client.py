#! /user/bin/env python

import rospy
import actionlib
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from moveit_commander import roscpp_initialize, RobotCommander


def pick_client():
    client = actionlib.SimpleActionClient('pick_custom',
                                          moveit_msgs.msg.PickupAction)

    client.wait_for_server()

    goal = moveit_msgs.msg.PickupActionGoal().goal
    #print goal
    goal.target_name = "box"
    print goal
    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


def add_objects(scene):

    rospy.sleep(2)
    scene.remove_world_object("base")
    scene.remove_attached_object("gripper_link","box")
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = "base_link"
    p.pose.position.x = 0.165
    p.pose.position.y = 0.
    p.pose.position.z = 0.066 - 0.115
    p.pose.orientation.x = 0.707106
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 0.707106
    scene.add_box("base", p, [0.052, 0.136, 0.526])
    p.header.frame_id = "base_link"
    p.pose.position.x = 0.165
    p.pose.position.y = 0.
    p.pose.position.z = 0.150 - 0.115
    p.pose.orientation.w = 1
    p.pose.orientation.x = 0
    scene.add_box("box", p, [0.03, 0.03, 0.03])
    rospy.sleep(1)
    rospy.loginfo("Objects added")


if __name__ == '__main__':
    try:
        roscpp_initialize([])
        rospy.init_node('pick_action_client')
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        add_objects(scene)
        result = pick_client()
        print result
    except rospy.ROSInterruptException:
        print('Progam interrupted')
