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

import numpy as np
import time
import re

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
        dataType=0
        returnCode, objects_handles, intData, floatData, objects_names=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)   
        
        # retrieves the object types
        dataType=1
        returnCode, objects_handles, objects_types, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)  
        
        # retrieves the parent object handles
        dataType=2
        returnCode, objects_handles,  parent_object_handles, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # retrieves the absolute object positions
        dataType=3
        returnCode, objects_handles,  parent_object_handles, objects_poses, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # retrieves the local object positions
        dataType=5
        returnCode, objects_handles,  parent_object_handles, objects_orientations, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 

        # get indexes of relevant elements in the warehouse
        obj_list = ['ConveyorBelt_forwarder', 'ConveyorBeltBody', 'Waypoint_CB', 'ShelfBody', 'TagShelf', 'DockStationBody', 'Bill', 'turtlebot2i', 'product', 'slidingDoor', 'stairs']
        obj_index_list = [objects_names.index(i) for i in objects_names if re.match(r'(#\d|)\b|'.join(obj_list)+'*', i)]


        print("objects_names: ", np.array(objects_names)[obj_index_list])
        print("objects_types: ", np.array(objects_types)[obj_index_list])
        print("objects_handles: ", np.array(objects_handles)[obj_index_list])
        print("objects_poses: ", np.reshape(np.array(objects_poses), (-1,3))[obj_index_list, :])
        print("objects_orientations: ", np.reshape(np.array(objects_orientations), (-1,3))[obj_index_list, :])


        for i in obj_index_list:
            # Get size of the obstacles (robot, worker, shelf, conveyor belt and product)
            returnCode, param_min_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 21, vrep.simx_opmode_blocking)
            returnCode, param_min_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 22, vrep.simx_opmode_blocking)
            returnCode, param_min_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 23, vrep.simx_opmode_blocking)
            returnCode, param_max_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 24, vrep.simx_opmode_blocking)
            returnCode, param_max_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 25, vrep.simx_opmode_blocking)
            returnCode, param_max_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 26, vrep.simx_opmode_blocking)

            print("object ", objects_names[i], " size x: ", param_max_x - param_min_x, " size y: ", param_max_y - param_min_y, " size z: ", param_max_z - param_min_z)

            # Get the velocity of the obstacle
            returnCode, param_vel_x = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 11, vrep.simx_opmode_blocking)
            returnCode, param_vel_y = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 12, vrep.simx_opmode_blocking)
            returnCode, param_vel_z = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 13, vrep.simx_opmode_blocking)
            returnCode, param_vel_r = vrep.simxGetObjectFloatParameter(clientID, objects_handles[i] , 14, vrep.simx_opmode_blocking)

            print("object ", objects_names[i], " vel x: ", param_vel_x, " vel y: ", param_vel_y, " vel z: ", param_vel_z, " vel r: ", param_vel_r)


#        print("objects_names: ",objects_names)
#        print("objects_types: ",objects_types)
#        print("objects_handles: ",objects_handles)
#        print("parent_object_handles: ",parent_object_handles)

    else:
        print ('Remote API function call returned with error code: ',res)




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
