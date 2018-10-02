#!/usr/bin/env python
'''
    Not sure whether we really need it.
    Prepare useful code segements here.
'''
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
'''  Lua code

myFunctionName=function(inInts,inFloats,inStrings,inBuffer)
-- inInts, inFloats and inStrings are tables
-- inBuffer is a string
-- Perform any type of operation here.
-- Always return 3 tables and a string, e.g.:
return {},{},{},''
end

'''
''' The remote API client Demo

inputInts=[1,2,3]
inputFloats=[53.21,17.39]
inputStrings=['Hello','world!']
inputBuffer=bytearray()
inputBuffer.append(78)
inputBuffer.append(42)

res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'objectName',vrep.sim_scripttype_childscript,
'myFunctionName',inputInts,inputFloats,inputStrings,inputBuffer,vrep.simx_opmode_blocking)
if res==vrep.simx_return_ok:
print (retInts)
print (retFloats)
print (retStrings)
print (retBuffer)

'''





# Create and start remote api connection with V-REP
def start_connection(ip, port):
    vrep.simxFinish(-1) 
    clientID = vrep.simxStart(ip, port, True, True, 5000, 5) 
    vrep.simxSynchronous(clientID, True)

    return clientID


clientID = start_connection('127.0.0.1', 19997)

objectName = "screenshotSensor"
screenshotFunctionName="screenshotRemoteAPI"
inputBuffer=bytearray()

result,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,objectName,vrep.sim_scripttype_customizationscript,screenshotFunctionName,[],[],[],inputBuffer,vrep.simx_opmode_blocking)
print result # 0 Success; 3 Failure

print (retStrings)
if result==vrep.simx_return_ok:
    print "Success"

else:
    print "Failed"
