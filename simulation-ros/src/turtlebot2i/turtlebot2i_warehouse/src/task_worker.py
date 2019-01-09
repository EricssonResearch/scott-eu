#!/usr/bin/env python

"""@package Task Worker
   Contains the classes to allocate the task to the robots 

"""

import yaml
import numpy as np
import rospy
import rospkg
import tf
import time
import actionlib
import threading
import queue
from collections import deque

import moveit_msgs.msg
import geometry_msgs.msg
from turtlebot2i_warehouse.msg import *
from turtlebot2i_warehouse.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from moveit_commander import PlanningSceneInterface
from moveit_commander import roscpp_initialize, RobotCommander
from tf.transformations import quaternion_from_euler


class TaskWorker:
    """Class with methods to get new tasks from task manager and execute.
       Runs inside robot.
    
    """
    def __init__(self, robot_name, scene):
        self.shelves = dict()
        self.products = dict()
        self.robot = robot_name
        self.scene = scene
        rospy.loginfo('Load moveit_commander')
        rospy.sleep(1)

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('turtlebot2i_warehouse')
        print(self.path)

        self.waypoints = self.load_waypoint(self.path + '/scene.yaml')
        self.load_objects(self.path + '/scene.yaml')

        self.storage = list(([[1.35, 1.02, 0.21], 0],
                             [[1.35, 0.96, 0.21], 0],
                             [[1.35, 0.90, 0.21], 0]))

        # This will be called just in the plan_callback
        #plan = self.get_plan('turtlebot2i')

        self.task_queue = queue.Queue()
        self.stopped = threading.Event()
        self.thread_interval = 1 # seconds

        self.prev_stamp = 0.0

        t = threading.Thread(target=self.plan_callback, args=(self.robot, self.thread_interval))
        t1 = threading.Thread(target=self.run_task, args=(self.thread_interval,))
        #t1 = threading.Thread(target=self.run_task)
        #t = threading.Timer(1, self.plan_callback, args=('turtlebot2i',))
        t.start()
        t1.start()

    def load_yaml(self, path):
        stream = file(path, 'r')
        return yaml.load(stream)

    def load_waypoint(self, path):
        yaml = self.load_yaml(path)

        waypoints = {}

        for line in yaml:
            waypoints[line['node']['name']] = line['node']['waypoint']

        return waypoints

    def to_pose(self, waypoint, frame="/map"):
        """Method to convert waypoint pose to ROS pose message.
    
        """
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = waypoint[0]
        p.pose.position.y = waypoint[1]
        p.pose.position.z = waypoint[2]
        # checks for quaternion and euler angles
        if len(waypoint) == 7:
            p.pose.orientation.x = waypoint[3]
            p.pose.orientation.y = waypoint[4]
            p.pose.orientation.z = waypoint[5]
            p.pose.orientation.w = waypoint[6]
        elif len(waypoint) == 6:
            q = quaternion_from_euler(waypoint[3]/180.0*np.pi,
                                      waypoint[4]/180.0*np.pi,
                                      waypoint[5]/180.0*np.pi)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
        return p

    def add_mesh(self, node):
        try:
            self.scene.add_mesh(node['name'],
                                self.to_pose(node['obj_position']),
                                self.path + '/' + node['mesh'])
            rospy.loginfo('New object added: %s', node['name'])
        except:
            rospy.logwarn('Failed to add object: %s %s',
                          node['name'],
                          self.path + '/' + node['mesh'])

    def add_box(self, node, size=[0.035, 0.035, 0.030]):
        try:
            self.scene.add_box(node['name'],
                               self.to_pose(node['obj_position']),
                               size)
            rospy.loginfo('New object added: %s', node['name'])
        except:
            rospy.logwarn('Failed to add product: %s',
                          node['name'])

    def add_product(self, shelf, product, idx=''):
        product = dict(self.products[product])
        product['name'] += idx
        pose = self.shelves[shelf]['obj_position']
        pose = np.asarray(pose[:3]) + np.asarray(product['obj_position'][:3])
        product['obj_position'] = np.hstack(
            (pose, product['obj_position'][3:]))
        self.add_box(product)
        return product['name']

    def obj_position(self, waypoint, offset):
        waypoint = np.asarray(waypoint)
        offset = np.asarray(offset)
        if waypoint.shape[0] != offset.shape[0]:
            rospy.logwarn('Waypoint: [%s] and Offset: [%s] shapes don\'t match',
                          waypoint,
                          offset)
            return waypoint
        return waypoint - offset
   
    # TODO: check if the methods will work 
    def load_objects(self, path):
        shelf_item = list()
        yaml = self.load_yaml(path)
        for line in yaml:
            if 'offset' in line['node'].keys():
                line['node']['obj_position'] = self.obj_position(line['node']['waypoint'],
                                                                 line['node']['offset'])
            else:
                line['node']['obj_position'] = line['node']['waypoint']
            if 'mesh' in line['node'].keys():
                self.add_mesh(line['node'])
            if 'position' in line['node']['name']:
                shelf_item.append(line['node'])
            if 'Shelf' in line['node']['name']:
                self.shelves[
                    line['node']['name']] = line['node']
            if ('product' in line['node']['name']) | ('position' in line['node']['name']):
                self.products[
                    line['node']['name']] = line['node']
#        for i in shelf_item:
#            for j in self.shelves:
#                self.add_product(j, i['name'], idx=j)


    def drop_position(self, place_handle, offset=[-0.7, 0.03, -1.15]):
        return np.asarray(self.waypoints[place_handle][:3]) + np.asarray(offset)

    def get_attached_product(self, product_name):
        return 'productRed'

    def waypoint2pick(self, waypoint, product):
        ''' waypoint information is always infront of productRed
        '''
        waypoint = np.asarray(waypoint)
        productRed_position = np.asarray(
            self.products['productRed']['obj_position'])
        product_position = np.asarray(
            self.products[product]['obj_position'])
        delta = (productRed_position - product_position)[:3]
        waypoint[:3] -= delta[:3]
        return waypoint

    def __move_pose(self, pose):
        # TODO: The client must be retrieved by the robot name
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = self.to_pose(pose)
        client.send_goal(goal)

        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            state = client.get_state()
        if state == 3:
            rospy.loginfo("Goal succeeded!")    
        
    def __move_task(self, robot, target):
        # TODO: The client must be retrieved by the robot name
        client = actionlib.SimpleActionClient(robot + '/move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = self.to_pose(self.waypoints[target])
        client.send_goal(goal)

        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            state = client.get_state()
        if state == 3:
            rospy.loginfo("Goal succeeded!")

    def __pick_task(self, target_obj="box"):
        client = actionlib.SimpleActionClient('/pick_custom',
                                              moveit_msgs.msg.PickupAction)
        client.wait_for_server()
        goal = moveit_msgs.msg.PickupActionGoal().goal
        goal.target_name = target_obj
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

    def __available_storage(self):
        for store in self.storage:
            if store[1] == 0:
                return store
        return False

    # TODO
    def check_task_status(self, robot):
        """Check the robot's task queue.
    
        """
        pass
    #    client = actionlib.SimpleActionClient(
    #        robot + '/move_base', MoveBaseAction)
    #    client.wait_for_server()

    #    wait = client.wait_for_result()

    #    if wait:
    #        state = client.get_state()
    #        # succeeded
    #        if state == GoalStatus.SUCCEEDED:
    #            return 1
    #        # not completed
    #        else:
    #            return 0
    #    # without task
    #    else:
    #        return -1

    # TODO: Store the robot status in diagnostic_msg format
    # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html
    def get_robot_status(self, robot):
        """Get the robot's status (battery, ).
    
        """
        pass


    def get_plan(self, robot):
        """Client side of the plan_service to get the plan
    
        """
        rospy.loginfo('Requesting plan for %s', robot)
        rospy.wait_for_service('plan_service')
        try:
            get_plan = rospy.ServiceProxy('plan_service', TaskArray)
            resp = get_plan(rospy.Time(self.prev_stamp), robot)
            self.prev_stamp = resp.stamp.to_sec()        # TODO: Check wether is empty (i.e. None)

            return resp.task_array

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def plan_callback(self, robot, interval):
        """Add tasks in the queue each `interval` seconds
    
        """
        while not self.stopped.wait(interval):
            print('plan_callback')
            task_list = self.get_plan(robot)

            if task_list is not None:
                self.task_queue.queue = deque(task_list)

    def run_task(self, interval): ## move this function to robot's task worker
        """Access the task queue and execute in the robot
            
        """

        lastWp = []

        while not self.stopped.wait(interval):
        #while not self.stopped.wait(1):
            print('run callback')

            # TODO: check if the task was already retrieved (must check timestamp)
            try:
                task = self.task_queue.get(False)
            except queue.Empty:
                continue

            print('task: ' + task.action)
            if task.action == 'move':
                rospy.loginfo('Move to [%s]', task.target)
                self.__move_task(self.robot, task.target)
                lastWP = self.waypoints[task.target]
                #if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED):
                #    status = 'SUCCEEDED'

            elif task.action == 'pick':
                rospy.loginfo('Moving to [%s] at [%s]',
                              task.product,
                              task.target)
                rospy.loginfo('From [%s] \n to [%s]',
                              lastWP,
                              self.waypoint2pick(lastWP, task.product)
                              )            
                self.__move_pose(
                    self.waypoint2pick(lastWP, task.product)
                )
                product_name = self.add_product(task.target, task.product)
                rospy.loginfo('Picking [%s] at [%s]',
                              task.product,
                              task.target)
                input()
                self.__pick_task(product_name)
                store = self.__available_storage()
                if not store:
                    rospy.info("No available Store")
                else:
                    rospy.loginfo("Storing product on [%s]", store[0])
                    res = self.__place_task(store[0])
                    store[1] = 1
                
            elif task.action == 'drop':
                rospy.loginfo('Dropping [%s] at [%s]',
                              task.product,
                              task.target)
                self.scene.remove_attached_object('base_footprint', task.product)
                self.__pick_task(task.product)
                rospy.loginfo('Dropping at position: [%s]',
                              self.drop_position(task.target))
                self.__place_task(self.drop_position(task.target))

            # TODO: mark the plan as executed
            # self.__check_task_status()

if __name__ == "__main__":
    # rospy.sleep(30)
    roscpp_initialize([])
    rospy.init_node('task_worker_py')
#    robot = RobotCommander()
#    scene = PlanningSceneInterface()
#    # rospy.sleep(15)
#    plan_int = PlanInterpreter(robot, scene)
#    # print(plan_int.plan)
#    # print(plan_int.waypoints)
#    # print(plan_int.waypoints[""])
#    plan_int.task_manager()

    task_worker = TaskWorker('turtlebot2i', None)

    rospy.spin()
