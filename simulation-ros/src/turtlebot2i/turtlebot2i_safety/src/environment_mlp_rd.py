#!/usr/bin/env python

import rospy
import actionlib
import random
import vrep
import time
import numpy as np
import geometry_msgs.msg
import std_msgs.msg
import os
from geometry_msgs.msg import Twist, Pose
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import Float64
from turtlebot2i_safety.msg import SafetyZone, VelocityScale, SafetyRisk
from collections import deque
from math import pi, sqrt, sin, cos, radians, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from shapely.geometry import Polygon, box, LineString, Point
from shapely.affinity import translate
#from matplotlib import pyplot as plt

class VrepManipulation():
    def __init__(self):
        self.scenarioNr = 0
        self.clientID   = vrep.simxStart('127.0.0.1', 20001, True, True, 5000, 5) 
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.model_location = self.dirPath.replace('turtlebot2i_safety/src', 'turtlebot2i_description/v-rep_model/warehouse_scene/vrep_models/turtlebot2i_for_training.ttm')
        returnCode, self.robot_handle = vrep.simxGetObjectHandle(self.clientID, 'turtlebot2i', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox  = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox', vrep.simx_opmode_blocking) 
        returnCode, self.ConcreteBox0 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#0', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox1 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#1', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox2 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#2', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox3 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#3', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox4 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#4', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox5 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#5', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox6 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#6', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox7 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#7', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox8 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#8', vrep.simx_opmode_blocking)
        returnCode, self.ConcreteBox9 = vrep.simxGetObjectHandle(self.clientID, 'ConcreteBox#9', vrep.simx_opmode_blocking) 

        returnCode, self.round1  = vrep.simxGetObjectHandle(self.clientID, '80cmHighPillar100cm', vrep.simx_opmode_blocking) 
        returnCode, self.round2  = vrep.simxGetObjectHandle(self.clientID, '80cmHighPillar100cm0', vrep.simx_opmode_blocking) 

        returnCode, self.conv  = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt', vrep.simx_opmode_blocking)
        returnCode, self.conv0 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#0', vrep.simx_opmode_blocking)
        returnCode, self.conv1 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#1', vrep.simx_opmode_blocking)
        returnCode, self.conv2 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#2', vrep.simx_opmode_blocking)
        returnCode, self.conv3 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#3', vrep.simx_opmode_blocking)
        returnCode, self.conv4 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#4', vrep.simx_opmode_blocking)
        returnCode, self.conv5 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#5', vrep.simx_opmode_blocking)
        returnCode, self.conv6 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#6', vrep.simx_opmode_blocking)
        returnCode, self.conv7 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#7', vrep.simx_opmode_blocking)
        returnCode, self.conv8 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#8', vrep.simx_opmode_blocking)
        returnCode, self.conv9 = vrep.simxGetObjectHandle(self.clientID, 'ConveyorBelt#9', vrep.simx_opmode_blocking)

    def setScenarioOriginal(self):
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox, -1,  np.array([ 7.0, 4.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox0, -1, np.array([ 1.0, 7.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox1, -1, np.array([-2.0,-3.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox2, -1, np.array([-3.0,-4.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox3, -1, np.array([ 5.5, 0.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox4, -1, np.array([ 5.5,-4.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox5, -1, np.array([ 7.0,-2.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox6, -1, np.array([7.25,-4.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox7, -1, np.array([ 9.0,-4.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox8, -1, np.array([ 7.5,-6.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox9, -1, np.array([-7.0,-6.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        #Round object
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.round1, -1, np.array([ 2.0,-4.0, 0.35]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.round2, -1, np.array([ 2.0,-6.5, 0.35]), vrep.simx_opmode_oneshot_wait)
        #conveyor belt
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv,  -1, np.array([ 1.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv0, -1, np.array([-1.0,-0.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv1, -1, np.array([-3.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv2, -1, np.array([-5.0,-0.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv3, -1, np.array([-7.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv4, -1, np.array([-4.5,-6.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv5, -1, np.array([ 0.0, 4.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv6, -1, np.array([-9.0, 5.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv7, -1, np.array([-9.0,-3.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv8, -1, np.array([-4.0, 5.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv9, -1, np.array([-7.0, 3.0, 0.113]), vrep.simx_opmode_oneshot_wait)

    def setScenarioMove1(self):
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox, -1,  np.array([ 6.5, 4.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox0, -1, np.array([-0.5, 7.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox1, -1, np.array([-4.0,-6.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox2, -1, np.array([-2.0,-2.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox3, -1, np.array([ 7.0, 0.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox4, -1, np.array([ 5.5,-6.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox5, -1, np.array([ 8.0,-2.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox6, -1, np.array([ 6.0,-3.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox7, -1, np.array([ 8.0,-5.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox8, -1, np.array([ 7.0,-4.5, 0.5]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.ConcreteBox9, -1, np.array([-8.5,-3.0, 0.5]), vrep.simx_opmode_oneshot_wait)
        #Round object
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.round1, -1, np.array([ 0.0,-3.0, 0.35]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.round2, -1, np.array([ 4.0,-6.0, 0.35]), vrep.simx_opmode_oneshot_wait)
        #conveyor belt
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv,  -1, np.array([ 2.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv0, -1, np.array([-2.0,-0.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv1, -1, np.array([-4.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv2, -1, np.array([-6.0,-0.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv3, -1, np.array([-8.0, 1.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv4, -1, np.array([-4.5,-4.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv5, -1, np.array([-4.0, 4.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv6, -1, np.array([-7.0, 2.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv7, -1, np.array([-9.0,-5.0, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv8, -1, np.array([0.25, 4.5, 0.113]), vrep.simx_opmode_oneshot_wait)
        returnCode = vrep.simxSetObjectPosition(self.clientID, self.conv9, -1, np.array([-9.0, 5.0, 0.113]), vrep.simx_opmode_oneshot_wait)

    def reset_robot_pos(self):
        print("RESET ROBOT POS REACHED")
        #reset robot position to origin
        
        returnCode = vrep.simxRemoveModel(self.clientID, self.robot_handle, vrep.simx_opmode_oneshot_wait)
        #print("Removing robot, robot handler:",self.robot_handle," | return code:",returnCode)
        returnCode, self.robot_handle = vrep.simxGetObjectHandle(self.clientID, 'turtlebot2i', vrep.simx_opmode_oneshot_wait)
        while(returnCode == 0):
            returnCode = vrep.simxRemoveModel(self.clientID, self.robot_handle, vrep.simx_opmode_oneshot_wait)
            rospy.loginfo("Previous removal failed. Remove robot again, robot handler:",self.robot_handle," | return code:",returnCode)
            returnCode, self.robot_handle = vrep.simxGetObjectHandle(self.clientID, 'turtlebot2i', vrep.simx_opmode_oneshot_wait)
        
        returnCode, self.robot_handle = vrep.simxLoadModel(self.clientID, self.model_location, 0, vrep.simx_opmode_oneshot_wait) 
        #print("Loading robot, robot handler:",self.robot_handle," | return code:",returnCode)
        while(returnCode != 0):
            returnCode, self.robot_handle = vrep.simxLoadModel(self.clientID, self.model_location, 0, vrep.simx_opmode_oneshot_wait) 
            rospy.loginfo("Previous loading failed. Reload robot. robot handler:",self.robot_handle," | return code:",returnCode)
        
    def remove_all_turtlebot2i(self):
        turtlebot2i_namelist = ['turtlebot2i', 'turtlebot2i#0', 'turtlebot2i#1', 'turtlebot2i#2', 'turtlebot2i#3', 'turtlebot2i#4', 'turtlebot2i#5', 'turtlebot2i#6', 'turtlebot2i#7', 'turtlebot2i#8', 'turtlebot2i#9',
                                'turtlebot_body_visual','turtlebot_reference','plate_middle_link_visual','plate_middle_link_respondable','GPS']
        for turtlebot2i_name in turtlebot2i_namelist:
            returnCode, temp_robot_handle = vrep.simxGetObjectHandle(self.clientID, turtlebot2i_name, vrep.simx_opmode_oneshot_wait)
            returnCode = vrep.simxRemoveModel(self.clientID, temp_robot_handle, vrep.simx_opmode_oneshot_wait)

    def check_robot_correctness(self):
        returnCode, self.robot_handle = vrep.simxGetObjectHandle(self.clientID, 'turtlebot2i', vrep.simx_opmode_oneshot_wait)
        while(returnCode != 0):
            rospy.loginfo("the exact 'turtlebot2i' is not found! Try to delete all possible robot and then load again.")
            self.remove_all_turtlebot2i()
            returnCode, self.robot_handle = vrep.simxLoadModel(self.clientID, self.model_location, 0, vrep.simx_opmode_oneshot_wait) 
            time.sleep(10)
            returnCode, self.robot_handle = vrep.simxGetObjectHandle(self.clientID, 'turtlebot2i', vrep.simx_opmode_oneshot_wait)
        

    def changeScenario(self):
        self.scenarioNr += 1
        if self.scenarioNr >= 2:
            self.scenarioNr = 0
        if self.scenarioNr == 0:
            self.setScenarioOriginal()
        elif self.scenarioNr == 1:
            self.setScenarioMove1()

    def shutdown(self):
        vrep.simxFinish(self.clientID)

class Env():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('turtlebot2i/move_base', MoveBaseAction)
        self.vrep_control = VrepManipulation()
        self.goal   = MoveBaseGoal()
        self.goal.target_pose.pose.position.x = 5.0 
        self.goal.target_pose.pose.position.y = 5.0 
        self.target_list = [[-9.0, 6.5],
                    [-9.0, 3.0],
                    [-4.0, 6.5],
                    [-4.0, 3.0],
                    [-0.5, 6.0],
                    [ 1.0, 3.0],
                    [ 5.0, 2.5],
                    [ 3.0, 0.0],
                    [-8.5, 0.0],
                    [-0.5,-2.0],
                    [ 9.0,-6.5],
                    [ 5.0,-2.0],
                    [-4.5,-2.5],
                    [ 8.5,-0.5],
                    [-9.0,-6.5]]
        self.target_idx = random.randrange(0, len(self.target_list))

        self.action_list = [[0.0, 0.0], [0.2, 0.2], [0.4, 0.4], [0.6, 0.6], [0.7, 0.7], [0.8, 0.8], [0.9, 0.9], [1.0, 1.0], [1.1, 1.1], [1.2, 1.2], [1.3, 1.3], [1.4, 1.4]]
        self.order = ["Progressive", "State", "ObsRWD", "SpeedRWD", "GoalRWD", "CollisionRWD", "DirectionRWD", "Total",
                      "NearestObs", "Direction", "LinSpeedBefore", "LinSpeedAfter", "LinSpeedScale", "RotSpeed",
                      "RiskMax", "Distance", "Done"]
        self.action_size = len(self.action_list)
        #self.get_goalbox = False
        self.position = Pose()
        self.prev_position = Pose()
        self.orientation = 0.0
        self.sub_pos        = rospy.Subscriber('/turtlebot2i/sensors/global_pose', geometry_msgs.msg.PoseStamped, self.update_pose_callback)
        self.sub_safetyzone = rospy.Subscriber('/turtlebot2i/safety/safety_zone', SafetyZone, self.safety_zone_callback)
        self.sub_vel        = rospy.Subscriber('/turtlebot2i/commands/velocity', Twist, self.speed_callback)
        self.sub_bumper     = rospy.Subscriber('/turtlebot2i/events/bumper', BumperEvent, self.bumper_callback)
        self.pub_safe_vel   = rospy.Publisher('/turtlebot2i/safety/vel_scale', VelocityScale, queue_size=10) #init publisher

        #Additional
        self.n_sensors = 675 #684 #if lidar on top: 684 data, if lidar in front of robot: 675 data

        self.robot_linear_speed  = 0.0 
        self.robot_angular_speed = 0.0
        self.statistics = [0] * 600
        self.analysis = []

        self.origin = Point((0.0, 0.0))
        self.camera_near_clipping = 0.2 #0.01 #in meters
        self.camera_far_clipping  = 3.5 #in meters
        self.camera_fov_angle     = 57.0 #in degree
        self.n_direction          = 12
        self.direction_list       = np.linspace(-self.camera_fov_angle, self.camera_fov_angle, self.n_direction+1)
        
        self.obstacle_map = []
        self.obstacle_distances = np.ones((self.n_direction))*self.camera_far_clipping
        for i in range(self.n_direction):
            self.obstacle_map.append(Polygon([[self.origin.x, self.origin.y],
                                    [self.camera_far_clipping*cos(radians(self.direction_list[i+1])),self.camera_far_clipping*sin(radians(self.direction_list[i+1]))],
                                    [self.camera_far_clipping*cos(radians(self.direction_list[i])),  self.camera_far_clipping*sin(radians(self.direction_list[i]))]]))

        #self.r_critical = 0.205
        #self.r_warning  = 0.31
        #self.r_clear    = 0.32
        self.collision  = False

        self.risk_max           = 0.0
        self.nearest_type       = 0
        self.min_distance       = self.camera_far_clipping
        self.nearest_direction  = 0.0
        self.nearest_speed      = 0.0

        self.speed_monitor = deque([])

    def distance2D(self, pos1, pos2):
        return sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def getGoalDistance(self):
        #print("goal pos:",self.goal.target_pose.pose.position,"\n robot pos:",self.position)
        return self.distance2D(self.goal.target_pose.pose.position, self.position)

    def update_pose_callback(self, data):
        self.prev_position = self.position
        self.position = data.pose.position
        (roll, pitch, self.orientation) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    def speed_callback(self, data):
        #getting data from move base module
        self.robot_linear_speed  = data.linear.x
        self.robot_angular_speed = data.angular.z 

    def safety_zone_callback(self, data):
        self.r_critical = data.critical_zone_radius
        self.r_warning  = data.warning_zone_radius
        self.r_clear    = data.clear_zone_radius

    def bumper_callback(self, data):
        if data.state == 1: #collision occurs
            self.collision = True

    def rotated_pos(self, pointX, pointY, centerX, centerY,r00, r01, r10, r11):
        point_X_rotated = r00*pointX + r01*pointY + centerX - r00*centerX - r01*centerY
        point_Y_rotated = r10*pointX + r11*pointY + centerY - r10*centerX - r11*centerY
        return [point_X_rotated, point_Y_rotated]

    def sceneGraphReconstruction(self, data):
        self.obstacle_distances = np.ones((self.n_direction))*self.camera_far_clipping
        n_obstacle = len(data.type) #count the number of detected object
        if n_obstacle > 0:
            self.risk_max = max(data.risk_value)
        else:
            self.risk_max = 0.0
        #fig = plt.figure(1, figsize=(3.5,6), dpi=90)
        #ax = fig.add_subplot(111)
        
        for i in range(n_obstacle):
            #### reconstruct the obstacle from scene graph ####
            obs_center_x = (data.distance[i])*cos(radians(data.direction[i]))
            obs_center_y = (data.distance[i])*sin(radians(data.direction[i]))
            
            r00 =  np.cos((-self.orientation))
            r01 = -np.sin((-self.orientation))
            r10 =  np.sin((-self.orientation))
            r11 =  np.cos((-self.orientation))
            obstacle = Polygon([self.rotated_pos(obs_center_x-data.size_x[i]/2, obs_center_y-data.size_y[i]/2, obs_center_x, obs_center_y, r00, r01, r10, r11),
                                self.rotated_pos(obs_center_x-data.size_x[i]/2, obs_center_y+data.size_y[i]/2, obs_center_x, obs_center_y, r00, r01, r10, r11),
                                self.rotated_pos(obs_center_x+data.size_x[i]/2, obs_center_y+data.size_y[i]/2, obs_center_x, obs_center_y, r00, r01, r10, r11),
                                self.rotated_pos(obs_center_x+data.size_x[i]/2, obs_center_y-data.size_y[i]/2, obs_center_x, obs_center_y, r00, r01, r10, r11)])
            curr_distance = self.origin.distance(obstacle) # need to be translated
            #print("distance to origin:",curr_distance,data.distance[i])
            obstacle = translate(obstacle, (data.distance[i]-curr_distance)*cos(radians(data.direction[i])), (data.distance[i]-curr_distance)*sin(radians(data.direction[i])))
            curr_distance = self.origin.distance(obstacle) # need to be translated
            #print("distance to origin2:",curr_distance,data.distance[i])
            while(data.distance[i] - curr_distance) > 0.02: #translate again if the distance is not close to the real distance
                obstacle = translate(obstacle, (data.distance[i]-curr_distance)*cos(radians(data.direction[i])), (data.distance[i]-curr_distance)*sin(radians(data.direction[i])))
                curr_distance = self.origin.distance(obstacle) 
                #print("distance to origin3:",curr_distance,data.distance[i])
            #x,y = obstacle.exterior.xy
            #ax.plot(x, y)
            for i in range(self.n_direction):
                #x,y = self.obstacle_map[i].exterior.xy
                #ax.plot(x, y)
                if obstacle.intersects(self.obstacle_map[i]):
                    intersection_poylgon = obstacle.intersection(self.obstacle_map[i])
                    #xC,yC= intersection_poylgon.exterior.xy
                    #ax.plot(xC, yC)
                    self.obstacle_distances[i] = min(self.obstacle_distances[i], self.origin.distance(intersection_poylgon))

        
        #print("obstacle_distances: ", self.obstacle_distances)
        #print("argmin_distance:",np.argmin(self.obstacle_distances))
        #plt.show() 

        return self.obstacle_distances


    def getState(self, safety_risk_msg):
        obstacle_distances = list(self.sceneGraphReconstruction(safety_risk_msg))
        done = False

        if self.collision:
            done = True
            self.collision = False

        if self.getGoalDistance() < 0.5:
            print("Goal reached!")
            #self.get_goalbox = False
            self.respawn_goal(reset=True)
        return obstacle_distances + [self.robot_linear_speed, self.robot_angular_speed, self.risk_max, self.r_warning, self.r_clear], done

    def publishScaleSpeed(self, left_vel_scale, right_vel_scale):
        vel_scale_message = VelocityScale()
        vel_scale_message.header = std_msgs.msg.Header()
        vel_scale_message.header.stamp = rospy.Time.now()
        vel_scale_message.left_vel_scale  = left_vel_scale
        vel_scale_message.right_vel_scale = right_vel_scale
        self.pub_safe_vel.publish(vel_scale_message)

    def respawn_goal(self, reset=False):
        if reset:
            self.vrep_control.changeScenario()

        #self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        #create a move base goal message
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        #choosing position randomly
        next_target_idx = random.randrange(0, len(self.target_list))
        while (self.target_idx == next_target_idx):
            next_target_idx = random.randrange(0, len(self.target_list))
        self.target_idx = next_target_idx
        next_goal = self.target_list[self.target_idx]
        self.goal.target_pose.pose.position.x = next_goal[0] 
        self.goal.target_pose.pose.position.y = next_goal[1] 
        self.goal.target_pose.pose.position.z = 0.063 
        #choosing orientation randomly
        orientation=geometry_msgs.msg.Quaternion()
        yaw  = random.uniform(-pi, pi)#-90*pi/180 #unit: from deg. to rad.
        orientation=quaternion_from_euler(0,0,yaw)#(roll, pitch,yaw) # return an array
        self.goal.target_pose.pose.orientation.x=0.0
        self.goal.target_pose.pose.orientation.y=0.0
        self.goal.target_pose.pose.orientation.z=orientation[2]
        self.goal.target_pose.pose.orientation.w=orientation[3]

        self.client.send_goal(self.goal)
        #rospy.loginfo("Goal position is sent! waiting the robot to finish....") 


    def setReward(self, state, done, action, i):
        nearest_obstacle_distance  = min(state[:12])
        direction = np.argmin(state[:12])
        #nearest_obstacle_direction = np.argmin(state[:12]) #index 0 start from right side of the robot

        # yaw_reward = 1.0
        # if (nearest_obstacle_direction <= self.n_direction/3-1):#obstacle is on the right
        #     if (action >= 10):                    #robot turns right
        #         yaw_reward = -(action-9)*risk_max/6
        # elif (nearest_obstacle_direction >= self.n_direction*2/3):#obstacle is on the left
        #     if (action <= 5):                   #robot turns left
        #         yaw_reward = -(6-action)*risk_max/6
        # else:#obstacle is in the front
        #     if (action in [6,7,8,9]):
        #         yaw_reward = -(action-5)*risk_max/4
        
        #distance_rate = 1.0 / max(nearest_obstacle_distance, 0.175)
        reward = 0
        clear_zone = state[-1]
        warning_zone = state[-2]
        linSpeed = state[-5]
        rotSpeed = state[-4]
        maxRisk = (state[-3] + 2) / 3
        scaling = self.action_list[action][0]
        scaled_speed = linSpeed * scaling
        scaled_speed_factor = scaled_speed

        # if nearest_obstacle_distance < self.r_critical + 0.03: #r_critical + offset
        #     reward = (distance_rate) * 10 - 100
        #     self.statistics[1] = self.statistics[1] + 1
        #     if state[-5] * scaling > 0.2:
        #         self.statistics[11] = self.statistics[11] + 1
        #         reward = - 50
        #     else:
        #         self.statistics[12] = self.statistics[12] + 1
        #         reward = reward + 20
        # elif nearest_obstacle_distance < state[-2] + 0.05: #r_warning state[-2] + offset
        #     self.statistics[2] = self.statistics[2] + 1
        #     reward =  - 30
        #     if state[-5] * scaling < 0.2 or state[-5] * scaling > 0.3:
        #         self.statistics[21] = self.statistics[21] + 1
        #         reward = reward - 15
        #     else:
        #         self.statistics[22] = self.statistics[22] + 1
        #         reward = reward + 20
        # elif nearest_obstacle_distance < state[-1]+ 0.05: #r_clear state[-1] + offset
        #     reward = - 20
        #     self.statistics[3] = self.statistics[3] + 1
        #     if state[-5] * scaling < 0.3 or state[-5] * scaling > 0.4:
        #         self.statistics[31] = self.statistics[31] + 1
        #         reward = reward - 15
        #     else:
        #         self.statistics[32] = self.statistics[32] + 1
        #         reward = reward + 20
        # else:
        #     self.statistics[4] = self.statistics[4] + 1
        #     reward = 10
        #     if state[-5] * scaling < 0.4:
        #         self.statistics[41] = self.statistics[41] + 1
        #         reward = reward - 25
        #     else:
        #         self.statistics[42] = self.statistics[42] + 1
        #         reward = reward + 20
        #
        # if self.distance2D(self.prev_position, self.position) > 0.03:
        #     self.statistics[5] = self.statistics[5] + 1
        #     reward = reward +  10
        # self.prev_position = self.position

        if i == 0:  # obstacle avoidance
            self.statistics[1] = self.statistics[1] + 1
            if nearest_obstacle_distance <= self.r_critical:
                self.statistics[11] = self.statistics[11] + 1
                if scaled_speed > 0.2:
                    self.statistics[111] = self.statistics[111] + 1
                    reward = -20 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[112] = self.statistics[112] + 1
                    reward = +20 * maxRisk / (scaled_speed_factor + 1)
            elif nearest_obstacle_distance <= warning_zone:
                self.statistics[12] = self.statistics[12] + 1
                if scaled_speed  <= 0.2:
                    self.statistics[121] = self.statistics[121] + 1
                    reward = -50 / maxRisk / (scaled_speed_factor + 1)
                elif scaled_speed > 0.25:
                    self.statistics[122] = self.statistics[122] + 1
                    reward = -7 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[123] = self.statistics[123] + 1
                    reward = +20 * maxRisk * (scaled_speed_factor + 1)
            elif nearest_obstacle_distance <= clear_zone:
                self.statistics[13] = self.statistics[13] + 1
                if scaled_speed <= 0.25:
                    self.statistics[131] = self.statistics[131] + 1
                    reward = -65 / maxRisk / (scaled_speed_factor + 1)
                elif scaled_speed > 0.35:
                    self.statistics[132] = self.statistics[132] + 1
                    reward = -5 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[133] = self.statistics[133] + 1
                    reward = +60 * (scaled_speed_factor + 1)
            else:
                self.statistics[14] = self.statistics[14] + 1
                if scaled_speed <= 0.35:
                    self.statistics[141] = self.statistics[141] + 1
                    reward = -75 / maxRisk / (scaled_speed_factor + 1)
                else:
                    self.statistics[142] = self.statistics[142] + 1
                    reward = +75 / maxRisk * (scaled_speed_factor + 1)

        elif i == 1:  # speed
            self.statistics[2] = self.statistics[2] + 1
            if scaled_speed >= 0.35:
                self.statistics[21] = self.statistics[21] + 1
                if nearest_obstacle_distance > clear_zone:
                    self.statistics[211] = self.statistics[211] + 1
                    reward = +65 / maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[212] = self.statistics[212] + 1
                    reward = -20 * maxRisk / (scaled_speed_factor + 1)
            elif scaled_speed >= 0.2 and scaled_speed < 0.35:
                self.statistics[22] = self.statistics[22] + 1
                if nearest_obstacle_distance <= clear_zone and nearest_obstacle_distance > self.r_critical:
                    self.statistics[221] = self.statistics[221] + 1
                    reward = + 25 * maxRisk * (scaled_speed_factor + 1)
                elif nearest_obstacle_distance > clear_zone:
                    self.statistics[222] = self.statistics[222] + 1
                    reward = -65 / maxRisk / (scaled_speed_factor + 1)
                else:
                    self.statistics[223] = self.statistics[223] + 1
                    reward = -15 * maxRisk * (scaled_speed_factor + 1)
            elif scaled_speed < 0.2:
                self.statistics[23] = self.statistics[23] + 1
                if nearest_obstacle_distance < self.r_critical:
                    self.statistics[231] = self.statistics[231] + 1
                    reward = +50 / maxRisk / (scaled_speed_factor + 1)
                else:
                    self.statistics[232] = self.statistics[232] + 1
                    reward = -25 * maxRisk * (scaled_speed_factor + 1)


        elif i == 2:  # getting closer to goal
            self.statistics[3] = self.statistics[3] + 1
            self.distance = self.distance2D(self.prev_position, self.position)
            if self.distance > 0.013:
                if nearest_obstacle_distance > clear_zone:
                    self.statistics[311] = self.statistics[311] + 1
                    reward = +75 / maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[312] = self.statistics[312] + 1
                    reward = -50 * maxRisk / (scaled_speed_factor + 1)
            elif self.distance >= 0.003 and self.distance < 0.013:
                if nearest_obstacle_distance > self.r_critical and nearest_obstacle_distance < clear_zone:
                    self.statistics[321] = self.statistics[321] + 1
                    reward = +50 * maxRisk / (scaled_speed_factor + 1)
                elif nearest_obstacle_distance < self.r_critical:
                    self.statistics[322] = self.statistics[322] + 1
                    reward = -50 * maxRisk * (scaled_speed_factor + 1)
                elif nearest_obstacle_distance > clear_zone:
                    self.statistics[323] = self.statistics[323] + 1
                    reward = -75 / maxRisk / (scaled_speed_factor + 1)
            elif self.distance < 0.003:
                if nearest_obstacle_distance < self.r_critical:
                    self.statistics[331] = self.statistics[331] + 1
                    reward = +50 * maxRisk / (scaled_speed_factor + 1)
                else:
                    self.statistics[332] = self.statistics[332] + 1
                    reward = -75 / maxRisk * (scaled_speed_factor + 1)
            self.prev_position = self.position

        elif i == 3:  # collision reward
            self.statistics[4] = self.statistics[4] + 1
            if done:
                self.statistics[41] = self.statistics[41] + 1
                rospy.loginfo("Collision!!")
                reward = reward - 5000
                self.publishScaleSpeed(1.0, 1.0)
                self.reset()
                self.prev_position = Pose()

        elif i==4: # direction reward
            self.statistics[5] = self.statistics[5] + 1
            if direction <= 3: # obstacle on the left
                self.statistics[51] = self.statistics[51] + 1
                if rotSpeed > 0.10:
                    self.statistics[511] = self.statistics[511] + 1
                    reward = +75 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[512] = self.statistics[512] + 1
                    reward = -60 * maxRisk * (scaled_speed_factor + 1)
            elif direction > 3 or direction <= 7: #obstacle in the center
                self.statistics[52] = self.statistics[52] + 1
                if rotSpeed < 0.10 or rotSpeed > -0.10:
                    self.statistics[521] = self.statistics[521] + 1
                    reward = -60 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[522] = self.statistics[522] + 1
                    reward = +75 * maxRisk * (scaled_speed_factor + 1)
            elif direction > 7: #obstacle on the right
                self.statistics[53] = self.statistics[53] + 1
                if rotSpeed < -0.10:
                    self.statistics[531] = self.statistics[531] + 1
                    reward = +75 * maxRisk * (scaled_speed_factor + 1)
                else:
                    self.statistics[532] = self.statistics[532] + 1
                    reward = -60 * maxRisk * (scaled_speed_factor + 1)
        # if self.get_goalbox:
        #    rospy.loginfo("Goal!!")
        #    self.respawn_goal(reset=True)
        #    self.get_goalbox = False

        print("Reward type " + str(i) + " : " + str(reward))

        return reward

    def step(self, action, active, n_models):
        if active:
            self.publishScaleSpeed(self.action_list[action][0], self.action_list[action][1])
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/turtlebot2i/safety/obstacles_risk', SafetyRisk, timeout=5)
            except:
                self.vrep_control.check_robot_correctness()
                pass

        state, done = self.getState(data)
        rewards = []
        if active:
            for i in range(n_models):
                rewards.append(self.setReward(state, done, action, i))
            element = {}
            element["Progressive"] = len(self.analysis) + 1
            element["State"] = state
            element["ObsRWD"] = rewards[0]
            element["SpeedRWD"] = rewards[1]
            element["GoalRWD"] = rewards[2]
            element["CollisionRWD"] = rewards[3]
            element["DirectionRWD"] = rewards[4]
            element["Total"] = sum(rewards)
            element["NearestObs"] = min(state[:12])
            element["Direction"] = np.argmin(state[:12])
            element["LinSpeedBefore"] = state[-5]
            element["LinSpeedAfter"] = state[-5] * self.action_list[action][0]
            element["LinSpeedScale"] = self.action_list[action][0]
            element["RotSpeed"] = state[-4]
            element["RiskMax"] = state[-3]
            element["Distance"] = self.distance
            element["Done"] = done
            self.analysis.append(element)


        return np.asarray(state), rewards, done

    def reset(self):
        self.publishScaleSpeed(0,0)
        self.vrep_control.reset_robot_pos()
        self.respawn_goal(reset=True)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/turtlebot2i/safety/obstacles_risk', SafetyRisk, timeout=5)
            except:
                self.vrep_control.check_robot_correctness()
                pass


        state, done = self.getState(data)

        return np.asarray(state)

