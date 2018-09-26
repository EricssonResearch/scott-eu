#!/usr/bin/env python
 
import json
import yaml
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from moveit_commander import roscpp_initialize, RobotCommander
import time

class Plan:
    def __init__(self):
        self.id = 0
        self.init = ''
        self.goal = ''
        self.task_list = list()


class Task:
    def __init__(self):
        self.action = ''
        self.robot = ''
        self.target = ''
        self.product = ''


class PlanInterpreter:
    # TODO: remove the hard coded part
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        rospy.loginfo('Load moveit_commander')
        rospy.sleep(1)
        self.waypoints = self.load_waypoint('/home/eznasam/project/scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_warehouse/scene.yaml')
        json_plan = self.load_json('/home/eznasam/project/scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_warehouse/plan.json')
        self.plan = self.parse_plan(json_plan)
        self.load_objects('/home/eznasam/project/scott-eu/simulation-ros/src/turtlebot2i/turtlebot2i_warehouse/scene.yaml')

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

    def load_objects(self, path):
        yaml = self.load_yaml(path) 
        for line in yaml:
            try:
                self.scene.add_mesh(
                    line['node']['name'],
                    line['node']['waypoint'],
                    line['node']['mesh'])
                rospy.loginfo('New object added: %s',
                      line['node']['name'])
                rospy.sleep(1)
            except:
                rospy.logwarn('Failed to add object: %s',
                      line['node']['name'])
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_footprint"
        p.pose.position.x = 0.165
        p.pose.position.y = 0.
        p.pose.position.z = 0.066 - 0.115
        p.pose.orientation.x = 0.707106
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0.707106
        rospy.loginfo(p)
        rospy.loginfo(self.scene.add_box("box", p, [0.03, 0.03, 0.03]))
        rospy.sleep(1)

    # def parse_plan(self, plan):
    def parse_plan(self, json_plan):
        p_plan = Plan()
        p_plan.id = json_plan['id']
        p_plan.init = json_plan['init']
        p_plan.goal = json_plan['goal']

        for task in json_plan['plan']:
            if (len(task)) >= 3:

                p_task = Task()
                p_task.action = task[0]
                p_task.robot = task[1]
                p_task.target = self.waypoint_to_cartesian(task[2])
                if (len(task)) == 4:
                    p_task.product = task[3]

                p_plan.task_list.append(p_task)

        return p_plan

    def task_manager(self):
        for task in self.plan.task_list:
            if task.action == 'move':
                print('move')
                # self.__move_task(task.robot, task.target)

            elif task.action == 'pick':
                print('pick')
                self.__pick_task()

            elif task.action == 'drop':
                print('drop')
                self.__place_task([0, 0, 0])

            # self.__check_task_status()

    def waypoint_to_cartesian(self, waypoint):
        return self.waypoints[waypoint]

    # def perform_task(self, plan):
#    def perform_task(self):
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
        client = actionlib.SimpleActionClient(
            robot + '/move_base', MoveBaseAction)
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

    def __pick_task(self, target_obj="box"):
        client = actionlib.SimpleActionClient('pick_custom',
                                              moveit_msgs.msg.PickupAction)
        client.wait_for_server()
        goal = moveit_msgs.msg.PickupActionGoal().goal
        goal.target_name = target_obj
        print goal
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def __place_task(self, target):
        client = actionlib.SimpleActionClient('place_custom',
                                              moveit_msgs.msg.PlaceAction)
        client.wait_for_server()
        goal = moveit_msgs.msg.PlaceActionGoal().goal
        goal.place_locations.append(moveit_msgs.msg.PlaceLocation())
        goal.place_locations[0].place_pose = geometry_msgs.msg.PoseStamped()
        goal.place_locations[0].place_pose.pose.position.x = target[0]
        goal.place_locations[0].place_pose.pose.position.y = target[1]
        goal.place_locations[0].place_pose.pose.position.z = target[2]
        print goal
        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()

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
    rospy.sleep(30)
    roscpp_initialize([])
    rospy.init_node('plan_interpreter_py')
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    rospy.sleep(15)
    plan_int = PlanInterpreter(robot, scene)
    #print(plan_int.plan)
    #print(plan_int.waypoints)
    #print(plan_int.waypoints[""])
    plan_int.task_manager()
