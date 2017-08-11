'''
RemoteApiInterface.py
@author: Ricaro Souza
@date : 18/04/2016

Description: Handles basic communication with V-REP remote API
'''
import sys, os
import middleware.sim.vrep as vrep
import time
import simplejson as json
import logging

log = logging.getLogger('middleware')

class RemoteApiInterface:

    def __init__(self, addr="127.0.0.1", port=19997):
        '''
        Init
        '''
        self.server_addr = addr
        self.empty_buff = bytearray()
        self.port = port
        self.connected = False
        log.info("Setting RemoteAPI configuration Addr: "  \
              + self.server_addr + ":" + str(self.port))


    def connect(self):
        '''
        Connect to remote API
        '''    
        vrep.simxFinish(-1)
        self.client_ID = vrep.simxStart(self.server_addr, self.port, 
                                       True, True, 5000, 5)
        if self.client_ID != -1:
           if vrep.simxStartSimulation(self.client_ID, 
                                       vrep.simx_opmode_blocking) == 0:
               log.info("Connected to V-REP simulator")
               return True, self.client_ID
        log.critical("Error connecting to V-REP simulator")
        self.connected = False
        return False, -1

        
    def disconnect(self):
        '''
        Disconnect from remote API
        '''
        vrep.simxStopSimulation(self.client_ID, vrep.simx_opmode_blocking)
        vrep.simxFinish(-1)
        log.info("Connection to simulator terminated")


    def get_client_ID(self):
        '''
        Return Remte API client ID
        '''
        return self.client_ID


    def get_elements(self):
        '''
        Get total number of objects on scene
        '''
        error_code, elements = vrep.simxGetObjects(self.client_ID,
                                            vrep.sim_handle_all,
                                            vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            log.debug("Total number of elements on the scene:", len(elements))
            return elements
        else:
            log.debug("Communication Error")
            return -1
    

    def get_points_of_interest_dict(self):
        '''
        Get Points of Interest Position data
        '''
        p_list = self._get_waypoints()
        if "Error" in p_list:
            return False #TODO: If this happens, the program crashes and produces the following error in other functions calling this one: "TypeError: 'bool' object is not iterable"
        p_list.sort()
        p_count = len(p_list)
        p_dict = {}

        for i in range(p_count):
            # TODO check response
            ret, pose = self.get_object_position(p_list[i])
            ret, name = self.get_object_name(p_list[i])
            p_dict[name] = pose
        return p_dict

    def get_robots(self):
        '''
        Get list of Robots
        '''
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, "Dummy",
                                          vrep.sim_scripttype_childscript,
                                          "getRobots", [], [], [], self.empty_buff,
                                          vrep.simx_opmode_oneshot_wait))
        
        if error_code is not 0:
            return "Error"
        return retStrings


    def _get_waypoints(self):
        '''
        Get List of Point of Interest    
        '''
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, "Dummy",
                                          vrep.sim_scripttype_childscript,
                                          "getPOI", [], [], [], self.empty_buff,
                                          vrep.simx_opmode_oneshot_wait))
        
        if error_code is not 0:
            return "Error"
        return retStrings


    def get_object_position(self, object_ID):
        '''
        Gets an object spatial position (x, y, z)
        '''
        error_code, handle = vrep.simxGetObjectHandle(self.client_ID, object_ID, 
                                                      vrep.simx_opmode_oneshot_wait)
        if error_code == 0:
            errot_code, pose = vrep.simxGetObjectPosition(self.client_ID, handle, -1, 
                                                          vrep.simx_opmode_oneshot_wait)
            return True, pose
        
        return False


    def get_object_name(self, object_ID):
        '''
        Get object name (Shelf, Conveyor Belt, etc)
        '''
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, "Dummy", 
                                     vrep.sim_scripttype_childscript,
                                     "getName", [], [], [object_ID], 
                                     self.empty_buff, 
                                     vrep.simx_opmode_oneshot_wait))

        if error_code == 0:
            name = retStrings[0]
            return True, name

        return False, None
