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
import time
import re


class ObjectNode:
    def __init__(self, name, vtype, pose, ori, size, vel, bbox_min, bbox_max, handle):
        self.name = name
        self.vtype = vtype
        self.pose = pose
        self.ori = ori
        self.size = size
        self.vel = vel
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max
        self.handle = handle

class VisionSensor:
    def __init__(self, name, vtype, pose, ori, res_x, res_y, angle, angle_x, angle_y, far_clipping, near_clipping, handle, parent_handle):
        self.name = name
        self.vtype = vtype
        self.res_x = res_x
        self.res_y = res_y
        self.angle = angle
        self.angle = angle_x
        self.angle = angle_y
        self.far_clipping = far_clipping
        self.near_clipping = near_clipping
        self.handle = handle
        self.parent_handle = parent_handle

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
#--- Generate Scene graph from simulation ----------

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
        #http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetStringParameter
        
        objectType=vrep.sim_appobj_object_type 
        operationMode=vrep.simx_opmode_blocking

        # retrieves the object names
        dataType = 0
        returnCode, objects_handles, intData, floatData, objects_names=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)   
        
        # retrieves the object types
        dataType = 1
        returnCode, objects_handles, objects_types, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)  
        
        # retrieves the parent object handles
        dataType = 2
        returnCode, objects_handles,  parent_object_handles, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # retrieves the absolute object positions
        dataType = 3
        returnCode, objects_handles,  intData, objects_poses, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # retrieves the object orientations
        dataType = 5
        returnCode, objects_handles,  intData, objects_orientations, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # get indexes of relevant elements in the warehouse
        obj_list = ['stairs', 'slidingDoor', 'ConveyorBeltBody', 'Waypoint_CB', 'ShelfBody', 'TagShelf', 'DockStationBody', 'Bill', 'turtlebot2i', 'product', 'slidingDoor', 'stairs']
        obj_index_list = [objects_names.index(i) for i in objects_names if re.match(r'(#\d|)\b|'.join(obj_list)+'*', i)]


        # filter the necessary object poses and orientations
        obj_pos_list = np.reshape(np.array(objects_poses), (-1,3))
        obj_ori_list = np.reshape(np.array(objects_orientations), (-1,3))

#        print("objects_names: ", np.array(objects_names)[obj_index_list])
#        print("objects_types: ", np.array(objects_types)[obj_index_list])
#        print("objects_handles: ", np.array(objects_handles)[obj_index_list])
#        print("objects_poses: ", obj_pos_list[obj_index_list, :])
#        print("objects_orientations: ", obj_ori_list[obj_index_list, :])

        num_objects = len(obj_index_list)

        yaml_node_list = []
        obj_node_list = []

        dist_matrix = np.diag(np.ones(num_objects)) - 1

        for i in obj_index_list:

            # Get size of the obstacles (robot, worker, shelf, conveyor belt and product)
            returnCode, param_min_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 21, vrep.simx_opmode_blocking)
            returnCode, param_min_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 22, vrep.simx_opmode_blocking)
            returnCode, param_min_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 23, vrep.simx_opmode_blocking)
            returnCode, param_max_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 24, vrep.simx_opmode_blocking)
            returnCode, param_max_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 25, vrep.simx_opmode_blocking)
            returnCode, param_max_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 26, vrep.simx_opmode_blocking)
            #returnCode, param_min_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 15, vrep.simx_opmode_blocking)
            #returnCode, param_min_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 16, vrep.simx_opmode_blocking)
            #returnCode, param_min_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 17, vrep.simx_opmode_blocking)
            #returnCode, param_max_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 18, vrep.simx_opmode_blocking)
            #returnCode, param_max_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 19, vrep.simx_opmode_blocking)
            #returnCode, param_max_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 20, vrep.simx_opmode_blocking)

            size_x = param_max_x - param_min_x
            size_y = param_max_y - param_min_y
            size_z = param_max_z - param_min_z

            size = np.array([size_x, size_y, size_z])

            bbox_min = [obj_pos_list[i,0]-size_x/2.0, obj_pos_list[i,1]-size_y/2.0, obj_pos_list[i,2]-size_z/2.0]
            bbox_max = [obj_pos_list[i,0]+size_x/2.0, obj_pos_list[i,1]+size_y/2.0, obj_pos_list[i,2]+size_z/2.0]

            # Get the velocity of the obstacle
            returnCode, param_vel_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 11, vrep.simx_opmode_blocking)
            returnCode, param_vel_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 12, vrep.simx_opmode_blocking)
            returnCode, param_vel_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 13, vrep.simx_opmode_blocking)
            returnCode, param_vel_r = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 14, vrep.simx_opmode_blocking)

            vel = np.array([param_vel_x, param_vel_y, param_vel_z, param_vel_r])

            # Generate the topological map using YAML structure
            yaml_node_list.append({'node' : {'name' : objects_names[i],'waypoint' : [float(obj_pos_list[i,0]), float(obj_pos_list[i,1]), float(obj_pos_list[i,2]), float(obj_ori_list[i,0]), float(obj_ori_list[i,1]), float(obj_ori_list[i,2])],'edges' : [],'vertices' : []}})

            # Store the objects in a list
            obj_node_list.append(ObjectNode(objects_names[i], \
                                objects_types[i],\
                                np.reshape(obj_pos_list[i,:], (1, 3)),\
                                np.reshape(obj_ori_list[i,:], (1, 3)),\
                                size, vel, bbox_min, bbox_max, objects_handles[i]))


#            print("object ", objects_names[i], " size x: ", size_x, " size y: ", size_y, " size z: ", size_z)
#            print("object ", objects_names[i], " vel x: ", param_vel_x, " vel y: ", param_vel_y, " vel z: ", param_vel_z, " vel r: ", param_vel_r)
#            print("object ", objects_names[i], "pose x:", obj_pos_list[i,0], "pose y:", obj_pos_list[i,1], "pose z:", obj_pos_list[i,2])

#            print("objects_names: ",objects_names)
#            print("objects_types: ",objects_types)
#            print("objects_handles: ",objects_handles)
#            print("parent_object_handles: ",parent_object_handles)

        #############################################
        # Get/Calculate visual sensor (vs) properties
        #############################################

        vs_list = ['camera_rgb']
        vs_index_list = [objects_names.index(i) for i in objects_names if re.match(r'(#\d|)\b|'.join(vs_list), i)]
        
        print('list size: ', len(vs_index_list))

        vs_list = []

        for i in vs_index_list:
    
            returnCode, param_near_clipping = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 1000, vrep.simx_opmode_blocking)
            returnCode, param_far_clipping = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 1001, vrep.simx_opmode_blocking)
            returnCode, param_resolution_x = vrep.simxGetObjectIntParameter(clientID, objects_handles[i], 1002, vrep.simx_opmode_blocking)
            returnCode, param_resolution_y = vrep.simxGetObjectIntParameter(clientID, objects_handles[i], 1003, vrep.simx_opmode_blocking)
            returnCode, param_perspective_angle = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 1004, vrep.simx_opmode_blocking)

            ratio = param_resolution_x/param_resolution_y

            if (ratio > 1):
                param_perspective_angle_x = param_perspective_angle
                param_perspective_angle_y = 2*np.arctan(np.tan(param_perspective_angle/2)/ratio)
            else:
                param_perspective_angle_x = 2*np.arctan(np.tan(param_perspective_angle/2)*ratio)
                param_perspective_angle_y = param_perspective_angle

            vs_list.append(VisionSensor(objects_names[i], objects_types[i], obj_pos_list[i], obj_ori_list[i], param_resolution_x, param_resolution_y, param_perspective_angle, param_perspective_angle_x, param_perspective_angle_y, param_near_clipping, param_far_clipping, objects_handles[i], parent_object_handles[i]))

            print('name: ', objects_names[i], 'pos: ', obj_pos_list[i], 'ori: ', obj_ori_list[i], 'res x: ', param_resolution_x, 'res y: ', param_resolution_y, 'angle: ', param_perspective_angle, 'angle x: ', param_perspective_angle_x, 'angle y: ', param_perspective_angle_y)

            # TODO:Include a bias in the visual sensor orientation (90 degrees)
            bias = np.pi/2.0

            # Get the 2D polygon that represents the camera FOV
            
            #t = bias - obj_ori_list[i][1]
            #if (2*np.pi - t) % 2*np.pi > np.pi:
            #    t = -t

            #returnCode, r = vrep.simxGetObjectOrientation(clientID, parent_object_handles[i], -1, vrep.simx_opmode_streaming)

            #returnCode, p = vrep.simxGetObjectParent(clientID, objects_handles[i], vrep.simx_opmode_blocking)
            #returnCode, r = vrep.simxGetObjectOrientation(clientID, p, -1, vrep.simx_opmode_streaming)

            try:
                parent_name = 'turtlebot2i#' + objects_names[i].split('#')[1]
            except IndexError:
                parent_name = 'turtlebot2i'

            returnCode, parent_handler = vrep.simxGetObjectHandle(clientID, parent_name, vrep.simx_opmode_blocking)
            returnCode, r = vrep.simxGetObjectOrientation(clientID, parent_handler, -1, vrep.simx_opmode_blocking)
            
            t = -r[2] 
            #if t < 0:
            #    t = -t
            print('ph: ', parent_handler, ' ret: ', returnCode)
            print('ori: ', r)
            print('t: ', t)

            ## Perspective Projection
            H = param_far_clipping
            D = H * np.tan(param_perspective_angle_y/2)

            h = param_near_clipping
            d = h * np.tan(param_perspective_angle_y/2)

            ## TODO: Orthogonal Projection

            # Rotation center
            cx1 = obj_pos_list[i][0]
            cy1 = obj_pos_list[i][1]
            cx2 = obj_pos_list[i][0]
            cy2 = obj_pos_list[i][1]

            # Perspective Projection
            x1 = obj_pos_list[i][0] + H
            y1 = obj_pos_list[i][1] - D
            x2 = obj_pos_list[i][0] + H
            y2 = obj_pos_list[i][1] + D

            fov_upper_base_r = [[(x1-cx1)*np.cos(t) + (y1-cy1)*np.sin(t) + cx1, (y1-cy1)*np.cos(t) - (x1-cx1)*np.sin(t) + cy1], [(x2-cx2)*np.cos(t) + (y2-cy2)*np.sin(t) + cx2, (y2-cy2)*np.cos(t) - (x2-cx2)*np.sin(t) + cy2]]
            #fov_upper_base_r = [[(y1-cy1)*np.cos(t) - (x1-cx1)*np.sin(t) + cy1, (x1-cx1)*np.cos(t) + (y1-cy1)*np.sin(t) + cx1], [(y2-cy2)*np.cos(t) - (x2-cx2)*np.sin(t) + cy2, (x2-cx2)*np.cos(t) + (y2-cy2)*np.sin(t) + cx2]]
            #fov_upper_base_r = [[y1*np.cos(t) - x1*np.sin(t), -(x1*np.cos(t) + y1*np.sin(t))], [y2*np.cos(t) - x2*np.sin(t), -(x2*np.cos(t) + y2*np.sin(t))]]
            #fov_upper_base_r = [[x1*np.cos(t) - y1*np.sin(t), y1*np.cos(t) + x1*np.sin(t)], [x2*np.cos(t) - y2*np.sin(t), y2*np.cos(t) + x2*np.sin(t)]]
            #fov_upper_base_r = [[x1, y1], [x2, y2]]

            x1 = obj_pos_list[i][0] + h 
            y1 = obj_pos_list[i][1] - d
            x2 = obj_pos_list[i][0] + h
            y2 = obj_pos_list[i][1] + d

            fov_lower_base_r = [[(x1-cx1)*np.cos(t) + (y1-cy1)*np.sin(t) + cx1, (y1-cy1)*np.cos(t) - (x1-cx1)*np.sin(t) + cy1], [(x2-cx2)*np.cos(t) + (y2-cy2)*np.sin(t) + cx2, (y2-cy2)*np.cos(t) - (x2-cx2)*np.sin(t) + cy2]]

            print('boundaries: ', fov_upper_base_r, fov_lower_base_r)
    
            ###################################################
            # Check which objects are inside visual sensor FOV
            ###################################################

            pol_fov = Polygon([fov_upper_base_r[0], fov_upper_base_r[1], fov_lower_base_r[0], fov_lower_base_r[1]])

            for obj in obj_node_list:
                #pol_obj = Polygon([obj.bbox_min[0], obj.bbox_min[1]],  [obj.bbox_max[0], obj.bbox_min[1]], [obj.bbox_max[0], obj.bbox_max[1]], [obj.bbox_min[0], obj.bbox_max[1]]) 
                pol_obj = box(obj.bbox_min[0], obj.bbox_min[1],  obj.bbox_max[0], obj.bbox_max[1])

                #print('name: ', obj.name, ' coords: ', list(pol_obj.exterior.coords))
                #print('name: ', obj.name, ' pose: ', obj.pose, ' size: ', obj.size)

                if pol_fov.intersects(pol_obj):
                    print(obj.name)

    else:
        print ('Remote API function call returned with error code: ', res)





    # Persists the yaml object
    file = open('scene.yaml', 'w')
    file.write(yaml.dump(yaml_node_list))
    file.close()

    
    print(yaml.dump(yaml_node_list))

#--------------------------------------------------- 
    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Finished generating scene graph.',vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
