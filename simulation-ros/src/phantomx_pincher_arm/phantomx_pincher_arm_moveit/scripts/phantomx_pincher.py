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
        rospy.sleep(8)
        # TO-DO: wait for moveit.
        self.listener = tf.TransformListener()
        self.robot = RobotCommander()
        self.init_planner()
        self.gripper_dimensions = [0.02, 0.02, 0.02]

    def init_planner(self):
        # initialize the planner parameters
        self.robot.pincher_arm.set_start_state(RobotState())
        self.robot.pincher_arm.set_num_planning_attempts(10000)
        self.robot.pincher_arm.set_planning_time(7)
        self.robot.pincher_arm.set_goal_position_tolerance(0.01)
        self.robot.pincher_arm.set_goal_orientation_tolerance(0.1)

    def set_start_state_to_current_state(self):
        self.robot.pincher_arm.set_start_state_to_current_state()
        self.robot.pincher_gripper.set_start_state_to_current_state()

    def openGripper(self):
        # open gripper
        posture = dict()
        posture["PhantomXPincher_gripperClose_joint"] = 0.035
        self.robot.pincher_gripper.set_joint_value_target(posture)
        gplan = self.robot.pincher_gripper.plan()
        if len(gplan.joint_trajectory.points) == 0:
            return None
        return gplan

    def closeGripper(self):
        # close gripper
        posture = dict()
        posture["PhantomXPincher_gripperClose_joint"] = 0.008
        self.robot.pincher_gripper.set_joint_value_target(posture)
        gplan = self.robot.pincher_gripper.plan()
        if len(gplan.joint_trajectory.points) == 0:
            return None
        return gplan

    def to_pose(self, waypoint, frame="/map"):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = waypoint[0]
        p.pose.position.y = waypoint[1]
        p.pose.position.z = waypoint[2]
        p.pose.orientation.w = 1
        if len(waypoint) == 7:
            p.pose.orientation.x = waypoint[3]
            p.pose.orientation.y = waypoint[4]
            p.pose.orientation.z = waypoint[5]
            p.pose.orientation.w = waypoint[6]
        elif len(waypoint) == 6:
            q = tf.transformations.quaternion_from_euler(
                waypoint[3]/180.0*np.pi,
                waypoint[4]/180.0*np.pi,
                waypoint[5]/180.0*np.pi)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
        return p

    def target_to_frame(self, target,
                        frame_to="/arm_base_link",
                        frame_from="/map",
                        orientation=False):
        pose = self.to_pose(target, frame=frame_from)
        self.listener.waitForTransform(frame_from,
                                       frame_to,
                                       rospy.Time.now(),
                                       rospy.Duration(4))
        self.listener.getLatestCommonTime(frame_from, frame_to)
        pose = self.listener.transformPose(frame_to, pose)
        if orientation:
            return [pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w]
        
        return [pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z]

    def correct_grasp_position(self, obj_dimension, gripper_orientation):
        '''
        This function corrects the grasp position accordantly to the object
        dimensions, the gripper size (available size for the gripper grasp) and
        the gripper orientation.
        Input:
        - obj_dimension: list with object dimension in meters along x,y,z
        - gripper_orientation: [roll, pitch, yaw] in radians
        '''
        if len(obj_dimension) < 3:
            return np.asarray([0, 0, 0])
        obj_dimension = np.asarray(obj_dimension)
        #  rospy.loginfo("Obj Dimension [%s]", obj_dimension)
        delta = (obj_dimension - self.gripper_dimensions)/2.
        delta = list(delta)
        delta[2] = 0
        delta[1] = 0.01  # gripper_link

        # Tranformation Matrix from gripper to arm_base
        T_G_B = tf.transformations.euler_matrix(gripper_orientation[0],
                                                gripper_orientation[1],
                                                gripper_orientation[2])
        
        gripper_delta = T_G_B*np.asmatrix(delta+[1]).T
        gripper_delta = gripper_delta[:3]
        
        rospy.loginfo("Delta [arm_base_link] [%s]", gripper_delta)

        return np.asarray(gripper_delta).flatten()
        
    def ef_pose(self, target,
                dimension=[],
                orientation=[],
                attemps=10,
                frame='/map'):
        '''
        Here we try to verify if the target is in the arm range. Also, we
        try to orient the end-effector(ef) to nice hard-coded orientation
        Returns: planned trajectory
        '''
        self.robot.get_current_state()
        target = self.target_to_frame(target, frame_from=frame)
        # rospy.loginfo("Planing to [%s] on arm_frame", target)
                
        d = pow(pow(target[0], 2) + pow(target[1], 2), 0.5)
        if d > 0.3:
            rospy.loginfo("Too far. Out of reach")
            return None
        if len(orientation) > 0:
            rpy = tf.transformations.euler_from_quaternion(orientation)
            # rospy.loginfo('Robot target quaternion [%s]',
            #              np.rad2deg(orientation))
            rp = rpy[1]
        else:
            rp = np.pi/2.0 - np.arcsin((d-0.1)/.205)

        ry = np.arctan2(target[1], target[0])
        rospy.loginfo("Original target %s %s", target, np.rad2deg([0, rp, ry]))
        delta = self.correct_grasp_position(dimension, [0, rp, ry])
        target -= delta
        rospy.loginfo(target)
        ry = np.arctan2(target[1], target[0])
        rospy.loginfo(ry)
        rospy.loginfo("Target after correction %s %s",
                      target,
                      np.rad2deg([0, rp, ry]))
        attemp = 0
        again = True
        while (again and attemp < 10):
            again = False
            rp_a = attemp * ((0.05 + 0.05)*np.random.ranf() - 0.05) + rp
            ry_a = ry
            #  attemp * ((0.05 + 0.05)*np.random.ranf() - 0.05) + ry
            q = tf.transformations.quaternion_from_euler(0, rp_a, ry_a)
            rospy.loginfo([0, rp_a, ry_a])

            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = '/arm_base_link'
            p.pose.position.x = target[0]
            p.pose.position.y = target[1]
            p.pose.position.z = target[2]
            #  + np.abs(np.cos(rp))/50.0
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            rospy.loginfo("New target pose [%.4f, %.4f, %.4f] [d: %.4f, p: %.4f, y: %.4f]",
                          p.pose.position.x,
                          p.pose.position.y,
                          p.pose.position.z,
                          d,
                          rp_a,
                          ry_a)

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
        return self.robot.pincher_arm.execute(plan)

    def gripper_execute(self, plan):
        return self.robot.pincher_gripper.execute(plan)
    
