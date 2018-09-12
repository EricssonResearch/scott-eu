"""
Simulation Truck Interface
"""

#from middleware.sim.SceneManager import *
from middleware.sim.RemoteApiInterface import *
import logging

log = logging.getLogger('middleware')

trucks = {
    "Truck#0":"MiniTruckCargo",
    "Truck#1":"MiniTruckCargo#0",
    "Truck#2":"MiniTruckCargo#1"
}

class TruckVREP():

    def __init__(self, _remote_api_interface):
        '''
        Constructor
        '''
        self.client_ID = _remote_api_interface.get_client_ID()
        self.empty_buff = bytearray()

    def departure(self, truck_id):
        '''
        Truck departure
        '''
        script = trucks[truck_id]
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, script, 
                                     vrep.sim_scripttype_childscript,
                                     "truckDepart", [], [], [], 
                                     self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))

        log.debug("%s departure"%truck_id)
            
    def arrival(self, truck_id):
        '''
        Truck arrival
        '''
        script = trucks[truck_id]
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, script, 
                                     vrep.sim_scripttype_childscript,
                                     "truckArrive", [], [], [], 
                                     self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))

        log.debug("%s arrival"%truck_id)
