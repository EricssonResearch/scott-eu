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
from shapely.geometry import Point, MultiPolygon, Polygon, box
import numpy as np
import time
import re
from shapely.ops import cascaded_union

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

        # retrieves the absolute object positions
        dataType = 3
        returnCode, objects_handles,  intData, objects_poses, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)

        # retrieves the object orientations
        dataType = 5
        returnCode, objects_handles,  intData, objects_orientations, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)

        # get indexes of relevant elements in the warehouse
        ground_object = 'Floor10x15m' #'Floor10x10m'
        obj_list = [ground_object, 'stairs', 'slidingDoor', 'ConveyorBeltBody', 'ShelfBody', 'DockStationBody', 'product', 'ConcretBlock']

        obj_index_list = [objects_names.index(i) for i in objects_names if re.match(r'(#\d|)\b|'.join(obj_list)+'*', i)]

        obj_index_list += [objects_names.index(i) for i in objects_names if re.match('.*HighWall.*', i)]

        # filter the necessary object poses and orientations
        obj_pos_list = np.reshape(np.array(objects_poses), (-1,3))
        obj_ori_list = np.reshape(np.array(objects_orientations), (-1,3))

        num_objects = len(obj_index_list)

        yaml_node_list = []
        obj_node_list = {}

        dist_matrix = np.diag(np.ones(num_objects)) - 1

        for i in obj_index_list:

            # Get size of the obstacles (robot, worker, shelf, conveyor belt and product)
            #returnCode, param_min_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 15, vrep.simx_opmode_blocking)
            #returnCode, param_min_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 16, vrep.simx_opmode_blocking)
            #returnCode, param_min_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 17, vrep.simx_opmode_blocking)
            #returnCode, param_max_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 18, vrep.simx_opmode_blocking)
            #returnCode, param_max_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 19, vrep.simx_opmode_blocking)
            #returnCode, param_max_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 20, vrep.simx_opmode_blocking)
            returnCode, param_min_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 21, vrep.simx_opmode_blocking)
            returnCode, param_min_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 22, vrep.simx_opmode_blocking)
            returnCode, param_min_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 23, vrep.simx_opmode_blocking)
            returnCode, param_max_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 24, vrep.simx_opmode_blocking)
            returnCode, param_max_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 25, vrep.simx_opmode_blocking)
            returnCode, param_max_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i], 26, vrep.simx_opmode_blocking)

            size_x = param_max_x - param_min_x
            size_y = param_max_y - param_min_y
            size_z = param_max_z - param_min_z

            size = np.array([size_x, size_y, size_z])

            bbox_min = [round(obj_pos_list[i,0]-size_x/2.0, 1), round(obj_pos_list[i,1]-size_y/2.0, 1), round(obj_pos_list[i,2]-size_z/2.0, 1)]
            bbox_max = [round(obj_pos_list[i,0]+size_x/2.0, 1), round(obj_pos_list[i,1]+size_y/2.0, 1), round(obj_pos_list[i,2]+size_z/2.0, 1)]

            vel = np.array([0, 0, 0, 0])

            # Generate the topological map using YAML structure
            yaml_node_list.append({'node' : {'name' : objects_names[i],'waypoint' : [float(obj_pos_list[i,0]), float(obj_pos_list[i,1]), float(obj_pos_list[i,2]), float(obj_ori_list[i,0]), float(obj_ori_list[i,1]), float(obj_ori_list[i,2])],'edges' : [],'vertices' : [float(bbox_min[0]), float(bbox_min[1]), float(bbox_max[0]), float(bbox_max[1])]}})

            # Store the objects in a list
            obj_node_list[objects_names[i]] = ObjectNode(objects_names[i], \
                                              objects_types[i],\
                                              np.reshape(obj_pos_list[i,:], (1, 3)),\
                                              np.reshape(obj_ori_list[i,:], (1, 3)),\
                                              size, vel, bbox_min, bbox_max, objects_handles[i])


            print(obj_node_list[objects_names[i]].name)

    else:
        print ('Remote API function call returned with error code: ', res)

    ################
    # Build the map
    ################

    # Get scenario pose and size from floor object
    map_origin = obj_node_list[ground_object].bbox_min[0:2]
    map_width  = obj_node_list[ground_object].size[0]+0.1
    map_height = obj_node_list[ground_object].size[1]+0.1

    map_resolution = 0.050

    map_cells_x = int(round(map_width / map_resolution))
    map_cells_y = int(round(map_height / map_resolution))

    # Get top floor polygon
    floor_bbox_min = obj_node_list[ground_object].bbox_min[0:2]
    floor_bbox_max = obj_node_list[ground_object].bbox_max[0:2]
    floor_pol = box(floor_bbox_min[0], floor_bbox_min[1], floor_bbox_max[0], floor_bbox_max[1])
    obj_node_list.pop(ground_object)

    # PGM file header
    map_pgm = 'P2\n' + str(map_cells_x) + ' ' + str(map_cells_y) + '\n255\n'

    # Create a empty matrix
    map_grid = np.zeros((map_cells_y, map_cells_x))

    # Iterate through objects list to create a MultiPolygon
    # containing bounding boxes of all objects
    # Note that all objects will be represented by a box
    obj_pol_list = list()

    for obj in obj_node_list:
        #obj_pol_list.append(box(obj.bbox_min[0], obj.bbox_min[1], obj.bbox_max[0], obj.bbox_max[1]))

        #calculating the bounding box
        X_min = obj_node_list[obj].bbox_min[0]
        Y_min = obj_node_list[obj].bbox_min[1]
        X_max = obj_node_list[obj].bbox_max[0]
        Y_max = obj_node_list[obj].bbox_max[1]
        x_pos = obj_node_list[obj].pose[0][0]
        y_pos = obj_node_list[obj].pose[0][1]
        z_rot = obj_node_list[obj].ori[0][2]

        #rotation on each object (calc refer to: http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/matrix2d/)
        r00 =  np.cos(z_rot)
        r01 = -np.sin(z_rot)
        r10 =  np.sin(z_rot)
        r11 =  np.cos(z_rot)
        bbox_X_min_rotated = r00*X_min + r01*Y_min + x_pos - r00*x_pos - r01*y_pos
        bbox_Y_min_rotated = r10*X_min + r11*Y_min + y_pos - r10*x_pos - r11*y_pos
        bbox_X_max_rotated = r00*X_max + r01*Y_max + x_pos - r00*x_pos - r01*y_pos
        bbox_Y_max_rotated = r10*X_max + r11*Y_max + y_pos - r10*x_pos - r11*y_pos
        obj_pol_list.append(box(bbox_X_min_rotated, bbox_Y_min_rotated, bbox_X_max_rotated, bbox_Y_max_rotated))
        

    #obj_pol = MultiPolygon(obj_pol_list)
    obj_pol = cascaded_union(obj_pol_list)

    # Populate occupied point from MultiPolygon
    for j in range(map_cells_y-1,-1,-1):
        for i in range(map_cells_x):
            px = map_origin[0] + i*map_resolution
            py = map_origin[1] + j*map_resolution

            if obj_pol.intersects(Point(px,py)) or floor_pol.touches(Point(px,py)):
                map_pgm += '0 '
            else:
                map_grid[j,i] = 255
                map_pgm += '255 '

        map_pgm += '\n'

    # Persists the PGM map
    file = open('map_test1.pgm', 'w')
    file.write(map_pgm)
    file.close()

    # Persists the yaml object
    file = open('scene_test1.yaml', 'w')
    file.write(yaml.dump(yaml_node_list))
    file.close()

    #print(yaml.dump(yaml_node_list))
    print("Finished! ")
    print("map_origin: ",map_origin," | resolution: ",map_resolution)
    

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
