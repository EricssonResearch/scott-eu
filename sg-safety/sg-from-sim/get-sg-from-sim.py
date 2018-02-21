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

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
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

        dataType=0
        returnCode, objects_handles, intData, floatData, objects_names=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)   
        dataType=1
        returnCode, objects_handles, objects_types, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode)  
        dataType=2
        returnCode, objects_handles,  parent_object_handles, floatData, stringData=vrep.simxGetObjectGroupData(clientID,objectType,dataType,operationMode) 


        print("objects_names: ",objects_names)
        print("objects_types: ",objects_types)
        print("objects_handles: ",objects_handles)
        print("parent_object_handles: ",parent_object_handles)

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
