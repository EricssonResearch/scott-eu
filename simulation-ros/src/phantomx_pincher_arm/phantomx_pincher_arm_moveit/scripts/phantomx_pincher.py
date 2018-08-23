#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg
import tf
from moveit_commander import RobotCommander, roscpp_initialize
from moveit_msgs.msg import RobotState


class Phantomx_Pincher():
    def __init__(self):
        roscpp_initialize([])
        self.robot = RobotCommander()
        self.init_planner()

    def init_planner(self):
        # initialize the planner parameters
        self.robot.pincher_arm.set_start_state(RobotState())
        self.robot.pincher_arm.set_num_planning_attempts(10000)
        self.robot.pincher_arm.set_planning_time(5)
        self.robot.pincher_arm.set_goal_position_tolerance(0.01)
        self.robot.pincher_arm.set_goal_orientation_tolerance(0.5)

    def set_start_state_to_current_state(self):
        self.robot.pincher_arm.set_start_state_to_current_state()
        self.robot.pincher_gripper.set_start_state_to_current_state()
        

    def openGripper(self):
        #open gripper
        posture = dict()
        posture["PhantomXPincher_gripperClose_joint"] = 0.030
        self.robot.pincher_gripper.set_joint_value_target(posture)
        gplan = self.robot.pincher_gripper.plan()
        if len(gplan.joint_trajectory.points) == 0:
            return None
        return gplan

    def closeGripper(self):
        # close gripper
        posture = dict()
        posture["PhantomXPincher_gripperClose_joint"] = 0.015
        self.robot.pincher_gripper.set_joint_value_target(posture)
        gplan = self.robot.pincher_gripper.plan()
        if len(gplan.joint_trajectory.points) == 0:
            return None
        return gplan

    def ef_pose(self, target, attemps=10):
        # Here we try to verify if the target is in the arm range. Also, we
        # try to orient the end-effector(ef) to nice hard-coded orientation
        # Returns: planned trajectory
        self.robot.get_current_state()
        d = pow(pow(target[0], 2) + pow(target[1], 2), 0.5)
        if d > 3.0:
            rospy.loginfo("Too far. Out of reach")
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

            self.robot.pincher_arm.set_pose_target(p)
            planed = self.robot.pincher_arm.plan()
            if len(planed.joint_trajectory.points) == 0:
                rospy.loginfo("Motion plan failed, trying again")
                attemp += 1
                again = True
        if again:
            return None
        return planed

    def arm_execute(self, plan):
        self.robot.pincher_arm.execute(plan)

    def gripper_execute(self, plan):
        self.robot.pincher_gripper.execute(plan)
    
