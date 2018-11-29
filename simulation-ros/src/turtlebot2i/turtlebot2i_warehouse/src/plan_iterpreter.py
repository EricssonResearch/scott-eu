#!/usr/bin/env python

import json
import yaml

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Plan:
    def __init__(self):
        self.id = 0
        self.init = ''
        self.goal = ''
        self.task_list = list()

class Task:
    def __init__(self):
        self.action = ''
        self.robot  = ''
        self.target = ''
        self.product = ''

class PlanInterpreter:
    
    # TODO: remove the hard coded part
    def __init__(self):
        self.waypoints = self.load_waypoint('scene.yaml')
        json_plan = self.load_json('plan.json')
        self.plan = self.parse_plan(json_plan)

    def load_json(self, path):
        return json.load(open(path))

    def load_yaml(self, path):
        stream = file(path, 'r')
        return yaml.load(stream)

    def load_waypoint(self, path):
        yaml = self.load_yaml(path)

        waypoints = {}

        for line in yaml:
            waypoints[line['node']['name']] = line['node']['waypoint']

        return waypoints

    #def parse_plan(self, plan):
    def parse_plan(self, json_plan):
    
        p_plan = Plan()
        p_plan.id = json_plan['id']
        p_plan.init = json_plan['init']
        p_plan.goal = json_plan['goal']

        for task in json_plan['plan']:
        
            if (len(task)) >= 3:

                p_task = Task()

                p_task.action = task[0]
                p_task.robot  = task[1]
                p_task.target = self.waypoint_to_cartesian(task[2])
                
                if (len(task)) == 4:
                    p_task.product = task[3]

                p_plan.task_list.append(p_task)

        return p_plan

    def task_manager(self):
        
        for task in self.plan.task_list:
            if task.action == 'move':
                print('move')
                #self.__move_task(task.robot, task.target)

            elif task.action == 'pick':
                print('pick')
                #self.__pick_task()

            elif task.action == 'drop':
                print('drop')
                #self.__drop_task()

            #self.__check_task_status()


    def waypoint_to_cartesian(self, waypoint):
        return self.waypoints[waypoint]

    #def perform_task(self, plan):
#    def perform_task(self):
#        
#        if self.plan == 'move':
#            __move_task()
#
#        elif self.plan == 'pick':
#            __pick_task()
#
#        elif self.plan == 'drop':
#            __drop_task()

    def __move_task(self, robot, target):
        # TODO: The client must be retrieved by the robot name
        client = actionlib.SimpleActionClient(robot + '/move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        goal.target_pose.pose.position.z = target[2]
        goal.target_pose.pose.orientation.w = target[3]

        client.send_goal(goal)
        
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")


    def __pick_task(self):
        pass

    def __drop_task(self):
        pass

    def __check_task_status(self, robot):
        client = actionlib.SimpleActionClient(robot + '/move_base', MoveBaseAction)
        client.wait_for_server()

        wait = client.wait_for_result()

        if wait:
            state = client.get_state()
            # succeeded
            if state == GoalStatus.SUCCEEDED:
                return 1
            # not completed
            else:
                return 0
        # without task
        else:
            return -1

if __name__ == "__main__":

    plan_int = PlanInterpreter()
    #print(plan_int.plan)
    #print(plan_int.waypoints)
    #print(plan_int.waypoints[""])
    plan_int.task_manager()
