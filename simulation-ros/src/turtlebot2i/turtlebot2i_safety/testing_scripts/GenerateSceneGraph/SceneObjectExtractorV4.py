#!/usr/bin/env python

# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
#
# http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import yaml
from shapely.geometry import Polygon, box
import numpy as np


class SceneObject:
    def __init__(self, name, pose, ori, size, vel, bbox_min, bbox_max, handle):
        self.name = name
        self.pose = pose
        self.ori = ori
        self.size = size
        self.vel = vel
        # TODO: disassociate bbox 
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max
        self.handle = handle

    def __repr__(self):
        return self.name


class VisionSensor:
    def __init__(self, name, res_x, res_y, angle, angle_x, angle_y, far_clipping, near_clipping, handle):
        self.name = name
        self.res_x = res_x
        self.res_y = res_y
        self.angle = angle
        self.angle_x = angle_x
        self.angle_y = angle_y
        self.far_clipping = far_clipping
        self.near_clipping = near_clipping
        self.handle = handle
        self.fov = {'upper_base':[0,0], 'lower_base':[0,0]}

    def __repr__(self):
        return self.name


class Robot:
    def __init__(self, name, pose, ori, size, vel, bbox_min, bbox_max, handle):
        self.name = name
        self.pose = pose
        self.ori = ori
        self.size = size
        self.vel = vel
        # TODO: disassociate bbox 
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max
        self.vision_sensor = None
        self.handle = handle

    def __repr__(self):
        return self.name


class SceneObjectExtractor:
    def __init__(self, ip, port):
        # TODO: return error if could not connect
        # start connection
        #self.clientID = self.start_connection('127.0.0.1', 19997)
        self.clientID = self.start_connection(ip, port)

        # callback mode
        #self.operation_mode = vrep.simx_opmode_streaming
        #self.operation_mode = vrep.simx_opmode_buffer
        self.operation_mode = vrep.simx_opmode_oneshot_wait
        #self.operation_mode = vrep.simx_opmode_oneshot_split
        #self.operation_mode = vrep.simx_opmode_oneshot
        #self.operation_mode = vrep.simx_opmode_blocking
    
        # TODO: get objects dynamically
        # List of object names to retrieve information
        # For now it is hardcoded
        self.static_obj_name_list  = ['stairs', 'slidingDoor',
                                      'DockStationBody', 'DockStationBody#0',\
                                      'ConveyorBeltBody', 'ConveyorBeltBody#0', 'ConveyorBeltBody#1',\
                                      'ShelfBody#1', 'ShelfBody#0', 'ShelfBody',\
                                      'Wall_Door', 'Wall_Window'] 
                                      
        self.dynamic_obj_name_list = [] #,
                                    #   'productRed#1', 'productRed#2', 'productRed#3', \
                                    #   'productGreen#1', 'productGreen#2', 'productGreen#3',\
                                    #   'productYellow#1', 'productYellow#2', 'productYellow#3']
        self.robot_name_list = ['turtlebot2i'] #, 'turtlebot2i#1']
        
        # List of objects with attributes
        self.static_obj_list = []
        self.dynamic_obj_list = []
        self.robot_obj_list = []


    # Create and start remote api connection with V-REP
    def start_connection(self, ip, port):
        vrep.simxFinish(-1) 
        clientID = vrep.simxStart(ip, port, True, True, 5000, 5) 
        vrep.simxSynchronous(clientID, True)

        return clientID


    # Get information of all objects
    def get_all_objects_info(self):
        
        obj_name_list = self.static_obj_name_list + self.dynamic_obj_name_list + self.robot_name_list
        obj_list = []

        for obj_name in obj_name_list:
    
            returnCode, obj_handle = vrep.simxGetObjectHandle(self.clientID, obj_name, self.operation_mode)
    
            # Get pose and orientation 
            returnCode, obj_pose = vrep.simxGetObjectPosition(self.clientID, obj_handle, -1, self.operation_mode)
            returnCode, obj_ori = vrep.simxGetObjectOrientation(self.clientID, obj_handle, -1, self.operation_mode)
    
            # Get size
            returnCode, param_min_x = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 21, self.operation_mode)
            returnCode, param_min_y = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 22, self.operation_mode)
            returnCode, param_min_z = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 23, self.operation_mode)
            returnCode, param_max_x = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 24, self.operation_mode)
            returnCode, param_max_y = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 25, self.operation_mode)
            returnCode, param_max_z = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 26, self.operation_mode)
    
            size_x = param_max_x - param_min_x
            size_y = param_max_y - param_min_y
            size_z = param_max_z - param_min_z
    
            obj_size = np.array([size_x, size_y, size_z])
   
            bbox_min = [obj_pose[0]-size_x/2.0, obj_pose[1]-size_y/2.0, obj_pose[2]-size_z/2.0]
            bbox_max = [obj_pose[0]+size_x/2.0, obj_pose[1]+size_y/2.0, obj_pose[2]+size_z/2.0]

            # Get velocity
            returnCode, param_vel_x = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 11, self.operation_mode)
            returnCode, param_vel_y = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 12, self.operation_mode)
            returnCode, param_vel_z = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 13, self.operation_mode)
            returnCode, param_vel_r = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 14, self.operation_mode)
    
            obj_vel = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])
   
            # Add object to static, dynamic or robot list 
            if (obj_name in self.static_obj_name_list):
                obj = SceneObject(obj_name, obj_pose, obj_ori, obj_size, obj_vel, bbox_min, bbox_max, obj_handle)
                self.static_obj_list.append(obj)
            elif (obj_name in self.dynamic_obj_name_list):
                obj = SceneObject(obj_name, obj_pose, obj_ori, obj_size, obj_vel, bbox_min, bbox_max, obj_handle)
                self.dynamic_obj_list.append(obj)
            else:
                obj = Robot(obj_name, obj_pose, obj_ori, obj_size, obj_vel, bbox_min, bbox_max, obj_handle)
                self.get_robot_vision_sensor_info(obj)
                self.robot_obj_list.append(obj)
        
            print(obj_name, obj_size)

        return obj_list


    # Get robot's vision sensor info
    def get_robot_vision_sensor_info(self, robot):
        
        try:
            vision_sensor_name = 'camera_rgb#' + robot.name.split('#')[1]
        except IndexError:
            vision_sensor_name = 'camera_rgb'
       
        robot.vision_sensor = self.get_vision_sensor_info_from_name(vision_sensor_name)


    # Get information of a visual sensor (name)
    def get_vision_sensor_info_from_name(self, vision_sensor_name):
        
        returnCode, obj_handle = vrep.simxGetObjectHandle(self.clientID, vision_sensor_name, self.operation_mode)

        if (returnCode == 0):
            returnCode, param_near_clipping = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 1000, self.operation_mode)
            returnCode, param_far_clipping = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 1001, self.operation_mode)
            returnCode, param_resolution_x = vrep.simxGetObjectIntParameter(self.clientID, obj_handle, 1002, self.operation_mode)
            returnCode, param_resolution_y = vrep.simxGetObjectIntParameter(self.clientID, obj_handle, 1003, self.operation_mode)
            returnCode, param_perspective_angle = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 1004, self.operation_mode)

            ratio = param_resolution_x/param_resolution_y

            if (ratio > 1):
                param_perspective_angle_x = param_perspective_angle
                param_perspective_angle_y = 2*np.arctan(np.tan(param_perspective_angle/2)/ratio)
            else:
                param_perspective_angle_x = 2*np.arctan(np.tan(param_perspective_angle/2)*ratio)
                param_perspective_angle_y = param_perspective_angle

            return VisionSensor(vision_sensor_name, \
                                param_resolution_x, \
                                param_resolution_y, \
                                param_perspective_angle, \
                                param_perspective_angle_x, \
                                param_perspective_angle_y, \
                                param_near_clipping, \
                                param_far_clipping, \
                                obj_handle)


    # Get information of dynamic objects (not used yet)
    def get_dynamic_obj_info(self):

        dynamic_obj_list = []

        for obj_name in self.dynamic_obj_name_list:
            returnCode, obj_handle = vrep.simxGetObjectHandle(self.clientID, obj_name, self.operation_mode)

            # Get pose and orientation 
            returnCode, obj_pose = vrep.simxGetObjectPosition(self.clientID, obj_handle, -1, self.operation_mode)
            returnCode, obj_ori = vrep.simxGetObjectOrientation(self.clientID, obj_handle, -1, self.operation_mode)
        
            # Get the velocity of the obstacle
            returnCode, param_vel_x = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 11, self.operation_mode)
            returnCode, param_vel_y = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 12, self.operation_mode)
            returnCode, param_vel_z = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 13, self.operation_mode)
            returnCode, param_vel_r = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 14, self.operation_mode)

            obj_vel = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])
        
        self.dynamic_obj_list = dynamic_obj_list


    # Update information of dynamic objects
    def update_dynamic_obj_info(self):

        for dynamic_obj in (self.dynamic_obj_list + self.robot_obj_list):
            obj_handle = dynamic_obj.handle 

            # Get pose and orientation 
            returnCode, obj_pose = vrep.simxGetObjectPosition(self.clientID, obj_handle, -1, self.operation_mode)
            returnCode, obj_ori = vrep.simxGetObjectOrientation(self.clientID, obj_handle, -1, self.operation_mode)
        
            # Get the velocity of the obstacle
            returnCode, param_vel_x = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 11, self.operation_mode)
            returnCode, param_vel_y = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 12, self.operation_mode)
            returnCode, param_vel_z = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 13, self.operation_mode)
            returnCode, param_vel_r = vrep.simxGetObjectFloatParameter(self.clientID, obj_handle, 14, self.operation_mode)

            obj_vel = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])
            
            obj_size = dynamic_obj.size
   
            bbox_min = [obj_pose[0]-obj_size[0]/2.0, obj_pose[1]-obj_size[1]/2.0, obj_pose[2]-obj_size[2]/2.0]
            bbox_max = [obj_pose[0]+obj_size[0]/2.0, obj_pose[1]+obj_size[1]/2.0, obj_pose[2]+obj_size[2]/2.0]
       
            # TODO: must check if obj is really updating
            dynamic_obj.pose = obj_pose
            dynamic_obj.ori = obj_ori
            dynamic_obj.vel = obj_vel
            dynamic_obj.bbox_min = bbox_min
            dynamic_obj.bbox_max = bbox_max


    # Updated robot (vision sensor) pose and orientation
    def update_all_robots_vision_sensors_fov(self):

        for robot in self.robot_obj_list:

            vision_sensor = robot.vision_sensor

            returnCode, obj_pose = vrep.simxGetObjectPosition(self.clientID, vision_sensor.handle, -1, self.operation_mode)
            
            parent_name = robot.name

            returnCode, parent_handler = vrep.simxGetObjectHandle(self.clientID, parent_name, self.operation_mode)
            returnCode, r = vrep.simxGetObjectOrientation(self.clientID, parent_handler, -1, self.operation_mode)
            
            # Use robot orientation as the sensor's
            t = -r[2] 

            ## Perspective Projection
            H = vision_sensor.far_clipping
            D = H * np.tan(vision_sensor.angle_y/2)

            h = vision_sensor.near_clipping
            d = h * np.tan(vision_sensor.angle_y/2)

            ## TODO: Orthogonal Projection

            # Rotation center
            cx1 = obj_pose[0]
            cy1 = obj_pose[1]
            cx2 = obj_pose[0]
            cy2 = obj_pose[1]

            # Perspective Projection
            x1 = obj_pose[0] + H
            y1 = obj_pose[1] - D
            x2 = obj_pose[0] + H
            y2 = obj_pose[1] + D

            fov_upper_base_r = [[(x1-cx1)*np.cos(t) + (y1-cy1)*np.sin(t) + cx1, (y1-cy1)*np.cos(t) - (x1-cx1)*np.sin(t) + cy1], [(x2-cx2)*np.cos(t) + (y2-cy2)*np.sin(t) + cx2, (y2-cy2)*np.cos(t) - (x2-cx2)*np.sin(t) + cy2]]

            x1 = obj_pose[0] + h 
            y1 = obj_pose[1] - d
            x2 = obj_pose[0] + h
            y2 = obj_pose[1] + d

            fov_lower_base_r = [[(x1-cx1)*np.cos(t) + (y1-cy1)*np.sin(t) + cx1, (y1-cy1)*np.cos(t) - (x1-cx1)*np.sin(t) + cy1], [(x2-cx2)*np.cos(t) + (y2-cy2)*np.sin(t) + cx2, (y2-cy2)*np.cos(t) - (x2-cx2)*np.sin(t) + cy2]]

            vision_sensor.fov['upper_base'] = fov_upper_base_r
            vision_sensor.fov['lower_base'] = fov_lower_base_r


    # Get objects that are in the vision sensor FOV
    def get_objects_from_vision_sensor(self, vision_sensor):
    
        obj_list_detected = []

        obj_list = self.static_obj_list + self.dynamic_obj_list + self.robot_obj_list

        pol_fov = Polygon([vision_sensor.fov['upper_base'][0], vision_sensor.fov['upper_base'][1], vision_sensor.fov['lower_base'][0], vision_sensor.fov['lower_base'][1]])

        for obj in obj_list:
            pol_obj = box(obj.bbox_min[0], obj.bbox_min[1], obj.bbox_max[0], obj.bbox_max[1])

            if pol_fov.intersects(pol_obj):
                obj_list_detected.append(obj)

        return obj_list_detected

