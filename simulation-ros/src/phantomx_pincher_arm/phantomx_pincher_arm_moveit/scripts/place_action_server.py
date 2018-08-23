#!/usr/bin/env python
import sys
import numpy as np
import rospy
import actionlib
import moveit_msgs.msg
from  geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, roscpp_initialize
from moveit_commander import PlanningSceneInterface
from phantomx_pincher import Phantomx_Pincher

class Place(object):
    _feedback = moveit_msgs.msg.PlaceActionFeedback().feedback
    _result = moveit_msgs.msg.PlaceActionResult().result

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                moveit_msgs.msg.PlaceAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        rospy.loginfo('Action Service Loaded')
        self.robot = Phantomx_Pincher()
        rospy.loginfo('Moveit Robot Commander Loaded')
        self.scene = PlanningSceneInterface()
        rospy.loginfo('Moveit Planning Scene Loaded')
        rospy.loginfo('Place action is ok. Awaiting for connections')

    def get_target(self):
        target = [0.17, 0.10, 0.028]
        return target

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        sucess = True

        if len(self.scene.get_attached_objects()) < 1:
            rospy.loginfo("No object attached")
            self._as.set_preempted()
            self._result.error_code.val = -1
            sucess = False
            return None

        self.robot.set_start_state_to_current_state()
        self._feedback.state = "Planing to place pose"
        self._as.publish_feedback(self._feedback)
        target = [goal.place_locations[0].place_pose.pose.position.x,
                  goal.place_locations[0].place_pose.pose.position.y,
                  goal.place_locations[0].place_pose.pose.position.z]
        plan = self.robot.ef_pose(target)
        if plan is None:
            rospy.loginfo("Plan to place failed")
            self._as.set_preempted()
            self._result.error_code.val = -1
            sucess = False
            return None

        self._feedback.state = "Going to place the object"
        self._as.publish_feedback(self._feedback)
        self._result.trajectory_descriptions.append("Going to place the object")
        self._result.trajectory_stages.append(plan)
        self.robot.arm_execute(plan)
        rospy.sleep(7)
        
        self._feedback.state = "Removing object to be placed from the planning scene"
        self._as.publish_feedback(self._feedback)
        obj  = self.scene.get_attached_objects()
        link = obj[obj.keys()[0]].link_name
        obj = obj[obj.keys()[0]].object
        self.scene.remove_attached_object(link, obj.id)

        self._feedback.state = "Planning to open the gripper"
        self._as.publish_feedback(self._feedback)
        plan = self.robot.openGripper()
        if plan is None:
            rospy.loginfo("Open Gripper plan failed")
            self._as.set_preempted()
            self._result.error_code.val = -1
            sucess = False
            return None
        self._result.trajectory_descriptions.append('OpenGripper')
        self._result.trajectory_stages.append(plan)
        self._feedback.state = "Openning gripper"
        print self._feedback
        self.robot.gripper_execute(plan)
        rospy.sleep(1)
        self._as.publish_feedback(self._feedback)

        self._feedback.state = "Re-adding object"
        pose = PoseStamped()
        pose.pose = obj.primitive_poses[0]
        pose.header = obj.header
        self.scene.add_box(obj.id, pose,
                           obj.primitives[0].dimensions)
        self._as.publish_feedback(self._feedback)

        self._feedback.state = "Planing to retreat after place"
        self._as.publish_feedback(self._feedback)
        target[2]+=0.02
        plan = self.robot.ef_pose(target)
        if plan is None:
            rospy.loginfo("Plan to retreat failed")
            self._as.set_preempted()
            self._result.error_code.val = -1
            sucess = False
            return None

        self._feedback.state = "Retreating"
        self._as.publish_feedback(self._feedback)
        self._result.trajectory_descriptions.append("Retreat")
        self._result.trajectory_stages.append(plan)
        self.robot.arm_execute(plan)
        rospy.sleep(7)

        if sucess:
            self._result.error_code.val = 1
            rospy.loginfo(self._result)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
#    roscpp_initialize(sys.argv)
    rospy.init_node('place_custom')
    server = Place(rospy.get_name())
    rospy.spin()
