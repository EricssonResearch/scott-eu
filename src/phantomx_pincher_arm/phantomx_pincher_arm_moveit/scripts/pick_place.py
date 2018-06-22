#!/usr/bin/env python
import sys
import numpy as np
import rospy
import geometry_msgs.msg
import tf
from moveit_commander import RobotCommander, roscpp_initialize
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import RobotState
import time


def openGripper():
    posture = dict()
    posture["PhantomXPincher_gripperClose_joint"] = 0.030
    print posture
    return posture

def closeGripper():
    posture = dict()
    posture["PhantomXPincher_gripperClose_joint"] = 0.015
    print posture
    return posture
    
def ef_pose(target, arm, attemps = 10):

    # Here we try to verify if the target is in the arm range. Also, we
    # try to orient the end-effector(ef) to nice hard-coded orientation
    
    d = pow(pow(target[0], 2) + pow(target[1], 2), 0.5)
    if d > 3.0:
        print("Too far. Out of reach")
        return None
    rp = np.pi/2.0 - np.arcsin((d-0.1)/.205)
    ry = np.arctan2(target[1], target[0])

    attemp = 0
    again = True
    while (again and attemp < 10):
        again = False

        rp_a = attemp * ((0.05 + 0.05)*np.random.ranf() - 0.05) + rp
        ry_a = attemp * ((0.05 + 0.05)*np.random.ranf() - 0.05) + ry
        q = tf.transformations.quaternion_from_euler(0, rp_a, ry_a)
        print [0, rp_a, ry_a]

        p = geometry_msgs.msg.Pose()
        p.position.x = target[0]
        p.position.y = target[1]
        p.position.z = target[2]
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        target[2] += np.abs(np.cos(rp))/250.0

        arm.set_pose_target(p)
        planed = arm.plan()
        if len(planed.joint_trajectory.points) == 0:
            print "useless"
            attemp += 1
            again = True
    if again:
        return None
    return planed


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    # -----------------------------
    #  Add objecto to the scene
    #------------------------------
    
    rospy.sleep(2)
    scene.remove_world_object("base")
    scene.remove_attached_object("gripper_link", "box")

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


    # ------------------------------
    # Configure the planner
    # -----------------------------
    robot = RobotCommander()
    robot.pincher_arm.set_start_state(RobotState())

    #robot.pincher_arm.set_planner_id('RRTConnectkConfigDefault')
    robot.pincher_arm.set_num_planning_attempts(10000)
    robot.pincher_arm.set_planning_time(5)
    robot.pincher_arm.set_goal_position_tolerance(0.01)
    robot.pincher_arm.set_goal_orientation_tolerance(0.5)

    # -----------------------------
    #  Open gripper
    #------------------------------

    robot.get_current_state()
    robot.pincher_arm.set_start_state(RobotState())
    fechado = openGripper()
    robot.pincher_gripper.set_joint_value_target(fechado)
    gplan = robot.pincher_gripper.plan()
    robot.pincher_gripper.execute(gplan)

    # -----------------------------
    #  Go to some pose
    #------------------------------
    robot.get_current_state()
    robot.pincher_arm.set_start_state(RobotState())
    target = [0.17, 0.005, 0.019]
    plan = ef_pose(target, robot.pincher_arm)


    if plan is not None:
        robot.pincher_arm.execute(plan)
        print(plan)
    else:
        print('No trajectory found')
        exit()


    rospy.sleep(5)
    scene.remove_world_object("box") #it's necessary to properly close the gripper
    rospy.sleep(1)

    # -----------------------------
    # Close gripper
    #------------------------------
    robot.get_current_state()
    robot.pincher_arm.set_start_state(RobotState())
    fechado = closeGripper()
    robot.pincher_gripper.set_joint_value_target(fechado)
    gplan = robot.pincher_gripper.plan()
    robot.pincher_gripper.execute(gplan)


    #with closed gripper, attach the box to it
    rospy.sleep(1)
    scene.attach_box("gripper_link", "box", p, [0.03, 0.03, 0.03])
    rospy.sleep(1)
    
    # -----------------------------
    #  Go to some pose
    #------------------------------
    robot.get_current_state()
    robot.pincher_arm.set_start_state(RobotState())
    target = [0.17, 0.10, 0.028]
    plan = ef_pose(target, robot.pincher_arm)
    
    if plan is not None:
        robot.pincher_arm.execute(plan)
        print(plan)
    else:
        print('No trajectory found')
        exit()

    rospy.sleep(5)
    scene.remove_attached_object("gripper_link", "box")#it's necessary to properly open the gripper
    rospy.sleep(1)
    scene.remove_world_object("box")#just because the program it's about to end
    rospy.sleep(1)

    # -----------------------------
    #  Open gripper
    #------------------------------
    robot.get_current_state()
    robot.pincher_arm.set_start_state(RobotState())
    fechado = openGripper()
    robot.pincher_gripper.set_joint_value_target(fechado)
    gplan = robot.pincher_gripper.plan()
    robot.pincher_gripper.execute(gplan)
