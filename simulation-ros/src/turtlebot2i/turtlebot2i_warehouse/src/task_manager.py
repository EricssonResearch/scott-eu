#!/usr/bin/env python

"""@package Task Manager
   Contains the classes to allocate PDDL plans to robots and monitor the robot task

"""

import json
import numpy as np
import rospy
import rospkg
import tf
import time
import actionlib

# TODO: Check if all these msgs are necessary
import moveit_msgs.msg
import geometry_msgs.msg
from turtlebot2i_warehouse.msg import *
from turtlebot2i_warehouse.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from moveit_commander import PlanningSceneInterface
from moveit_commander import roscpp_initialize, RobotCommander
from tf.transformations import quaternion_from_euler
# TODO: refactor classes

# TODO: Deprecate this class?
class Plan:
    def __init__(self):
        self.id = 0
        self.init = ''
        self.goal = ''
        self.task_list = list()

# TODO: Add robot id? Replace target by coordinates?
# TODO: make a standard name
class Task:
    def __init__(self):
        self.action = ''
#        self.robot = ''
        self.target = ''
        self.product = ''

class TaskList:
    """Class to store tasks grouped by robot
    
    """
    def __init__(self):
        self.stamp = None
        self.robot_tasks = {}

# Used in the TaskWorker
class Robot:
    def __init__(self):
        self.name = ''
        self.battery = -1
        self.task_list = list()
        self.task_status = ''


# Runs inside warehouse controller
class PlanInterpreter:
    """Class with methods to parse the plans and wrap in TaskList structure
    
    """

    def load_json(self, path):
        return json.load(open(path))

    def parse_plan(self, path):
        p_task_list = TaskList()
        json_plan = self.load_json(path)

        p_task_list.stamp = float(json_plan['stamp'])
        for p_plan in json_plan['plan']:
            if len(p_plan) >= 3:

                robot = p_plan[1]

                if robot not in p_task_list.robot_tasks:
                    p_task_list.robot_tasks[robot] = []

                task = Task()
                task.action = p_plan[0]
                task.target = p_plan[2]
                if (len(p_plan)) == 4:
                    task.product = p_plan[3]

                p_task_list.robot_tasks[robot].append(task)

        return p_task_list

 
class TaskManager:
    """Class with methods to get new plans, send to the robots (task worker) and monitor the tasks
       Runs inside warehouse controller.
    
    """
    def __init__(self):
        self.interpreter = PlanInterpreter()

        # Get plan path
        rospack = rospkg.RosPack()
        self.plan_path = rospack.get_path('turtlebot2i_warehouse') + '/plan.json'

        # ROS service
        rospy.Service('plan_service', TaskArray, self.plan_service)

        # ROS publisher to get robot status
    #    rospy.Subscriber('topic_to_define', custom_robot_status_msg_to_define, self.task_worker_monitor)

        rospy.loginfo("Task manager started")

    #def append_robot_list(robot_name)
    #    self.robot_list

    def plan_service(self, request):
        """Gets plan and parses it through the plan interpreter
        
        """
        rospy.loginfo("Plan Requested")

        # Parse plan
        # TODO: use a timestamp in the plan file?
        # TODO: get the plan from a callback not from a file
        plan_list = self.interpreter.parse_plan(self.plan_path)

        response = TaskArrayResponse()
       
        if plan_list.stamp < request.stamp.to_sec():    # plan_list.stamp already comes in sec format
            return response
        
        #robot_plan_list = plan_list.robot_tasks[request.robot]
        robot_plan_list = plan_list.robot_tasks.get(request.robot, [])

        robot_plan_list_msg = []

        for t in robot_plan_list:
            action = Action()
            action.action = t.action
            action.product = t.product
            action.target = t.target
            robot_plan_list_msg.append(action)
            #robot_plan_list_msg.append(t)  # test this approach

        response.stamp = rospy.Time(plan_list.stamp)
        response.task_array = robot_plan_list_msg

        return response


    # TODO: check robot status each X seconds
    # Must use http://wiki.ros.org/diagnostic_aggregator
    def task_worker_monitor(self, robot, plan):
        """Makes possible monitor the task worker
        
        """
        pass


if __name__ == "__main__":
    rospy.init_node('task_manager_py')

    task_manager = TaskManager()

    rospy.spin()
