"""
YouBot
@author Ricardo Souza
@date 21-06-2016

Description: Implements abstract class RobotBase
"""

from middleware.sim.RemoteApiInterface import *
from middleware.sim.logistics.RobotBase import *
import abc
import middleware.sim.vrep
import time
import math
import simpy
import logging

log = logging.getLogger('middleware')

position = None

class YouBot(RobotBase):
    """
    YouBot (V-REP) class - implements RobotBase
    """

    def __init__(self, _remote_api_interface, _warehouse_scene, robot_ID="youBot"):
        """
        Constructor
        """
        # get remote api connection
        self.client_ID = _remote_api_interface.get_client_ID()
        # create empty buffer for remoteAPI
        self.empty_buff = bytearray()
        # robot's ID
        self.id_ = robot_ID

        id_split = robot_ID.split('#')
        if len(id_split) > 1:
            index = id_split[1]
        else:
            index = None
        # robot's scripts

        self.interface_script_ID = "RemoteInterface"
        self.battery_script_ID = "Battery"
        if index is not None:
            self.interface_script_ID += "#"+str(index)
            self.battery_script_ID += "#"+str(index)

        # current scene
        self._scene = _warehouse_scene
        # robot logical position
        global position
        position = "Waypoint_RS#0"

    def get_id(self):
        return self.id_


    def move_to(self, waypoint_ID, async=False):
        """
        Move robot to waypoint (waypoint_ID)
        """

        # check if waypoint exists
        error_code, handle = vrep.simxGetObjectHandle(self.client_ID, waypoint_ID, 
                                              vrep.simx_opmode_oneshot_wait)
        if handle == 0:
            return False, json.loads('{"error":"waypoint not found"}')

        # send move command
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                          vrep.sim_scripttype_childscript, 
                                          'moveRobot', [], [], [waypoint_ID], 
                                          self.empty_buff, 
                                          vrep.simx_opmode_oneshot_wait))
        try:
            #verify return code
            if retInts[0] == 1:
                #check if asynchronous
                global position
                if async == True:
                    position = waypoint_ID
                    return True, json.loads('{"move_to":"'+retStrings[0]+'"}')
                else:
                    #verify if robot has completed the operation
                    ret_code, ret_msg = self.move_done()
                    while ret_msg["move"] == 'False':
                        ret_code, ret_msg = self.move_done()
                        time.sleep(0.5)
                    position = waypoint_ID
                return True, json.loads('{"move_to":"'+retStrings[0]+'"}')
            # error processing the command
            elif retInts[0] == 0:
                return True, json.loads('{"move_to":"'+retStrings[0]+'"}')
            # Unknown error
            else:
                log.debug("YouBotVRep.moveTo - unexpected return code" )
                return False, json.loads('{"error":"return code error"}')
        # Exceptions
        except ValueError:
            log.error( "YouBotVRep.moveTo - value error exception" )
            return False, json.loads('{"error":"return type not supported"}')
        except:
            log.error( "YouBotVRep.moveTo - internal error" )
            return False, json.loads('{"error":"internal error"}')

    def move_done(self):
        """ 
        Check robots movement 
        """
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                    vrep.sim_scripttype_childscript, \
                                    'checkRobotMovement', [], [], [], 
                                    self.empty_buff, 
                                    vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 0:
                return True, json.loads('{"move":"'+retStrings[0]+'"}')
            elif retInts[0] == 1:
                return True, json.loads('{"move":"'+retStrings[0]+'"}')
            else: 
                log.debug( "YouBotVRep.moveDone - unexpected return code" )
                return False, json.loads('{"error":"return code error"}')
        except ValueError:
            log.error( "YouBotVRep.moveDone - value error exception" )
            return False, json.loads('{"error":"return type not supported"}')
        except:
            log.error( "YouBotVRep.moveDone - internal error (exception)" )
            return False, json.loads('{"error":"internal error"}')


    def arm_move_done(self):
        """ 
        Check arm movement 
        """
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                    vrep.sim_scripttype_childscript, 
                                    'checkArmMovement', [], [], [], 
                                    self.empty_buff, 
                                    vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 0:
                return True, json.loads('{"arm":"False"}')
            elif retInts[0] == 1:
                return True, json.loads('{"arm":"True"}')
            else:
                log.debug("YouBotVrep.arm_move_done - unexpected return code")
                return False, json.loads('{"error":"return code error"}')
        except ValueError:
            log.error("YouBotVrep.arm_move_done - value error exception")
            return False, json.loads('{"error":"return type not supported"}')
        except:
            log.error("YouBOrVrep.arm_move_done - internal error (exception)")
            return False, json.loads('{"error":"internal error"}')

    def pick_up(self, object_ID, shelf_name=None):
        """ 
        Pick up object (object_ID) from shelf (shelf_name) 
        """
        
        # check if object and place exists
        error_code, obj_handle = vrep.simxGetObjectHandle(self.client_ID, object_ID, 
                                              vrep.simx_opmode_oneshot_wait)

        if obj_handle == 0:
            log.debug("Pick up: object not found")
            return False, json.loads('{"error":"object not found"}')

        # if shelf given get object ID
        if shelf_name != None:

            error_code, place_handle = vrep.simxGetObjectHandle(self.client_ID, shelf_name, 
                                              vrep.simx_opmode_oneshot_wait)
            if place_handle == 0:
                log.debug("Pick up: place not found")
                return False, json.loads('{"error":"place not found"}')

            # generate ID from name
            split_res = shelf_name.split('#')
            if len(split_res) > 1: # TODO all names have an index 
                shelf_ID = split_res[0] + "Body#" + split_res[1]
            else:
                shelf_ID = split_res[0] + "Body"

            # check if object pickable on given shelf
            obj = None
            _shelf = self._scene.get_shelf("std_shelf", "VREP")
            rcode, pickable_json = _shelf.get_pickable_list(shelf_ID)
            if not rcode:
                return False, pickable_json
            pickables = pickable_json["pickable"]
            for i in range(len(pickables)):
                if object_ID in pickables[i]:
                    obj = pickables[i]
                    break
            if obj == None:
                log.debug("Pick up: object not pickable")
                return False, json.loads('{"error":"object not pickable"}')
        # assume object ID complete
        else: # TO-DO check nearest shelf
            obj = object_ID

        # check if there is any empty slot
        rcode, current_cargo_json = self.get_cargo()
        empty = False
        empty_key = "free"
        cargo = current_cargo_json["cargo"]
        for i in range(len(cargo)):
            if empty_key in cargo[i]:
                empty = True
                break
        if empty == False:
            log.debug("Pick up: Robot is full")
            return False, json.loads('{"pickup":"no free slot"}')
        
        # send command to pick object
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                          vrep.sim_scripttype_childscript, 
                                          'pickup', [], [], [obj], 
                                          self.empty_buff, 
                                          vrep.simx_opmode_oneshot_wait))

        # check error_code
        try:
            if retInts[0] == 0:
                return True, json.loads('{"pickup":"'+retStrings[0]+'"}')
            elif retInts[0] == 1:
                # check if operation completed
                time.sleep(1)            
                #verify if robot has completed the operation
                ret_code, ret_msg = self.arm_move_done()
                while ret_msg["arm"] == 'True':
                    ret_code, ret_msg = self.arm_move_done()
                    time.sleep(0.1)
                return True, json.loads('{"pickup":"True"}')
            else:
                return False, json.loads('{"error":"'+retStrings[0]+'"}')
        except ValueError:
            return False, json.loads('{"error":"internal error"}')

    
    def drop_at(self, object_ID, place_ID):
        """ 
        Drop object (object_ID) on place (place_ID) 
        """

        # check object and place exists
        error_code, obj_handle = vrep.simxGetObjectHandle(self.client_ID, object_ID, 
                                              vrep.simx_opmode_oneshot_wait)

        place_ID = "Tag" + place_ID

        error_code, place_handle = vrep.simxGetObjectHandle(self.client_ID, place_ID, 
                                              vrep.simx_opmode_oneshot_wait)

        if obj_handle == 0 or place_handle == 0:
            return False, json.loads('{"error":"object not found"}')

        # check if product is loaded
        rcode, current_cargo_json = self.get_cargo()
        have_product = False
        cargo = current_cargo_json["cargo"]
        for i in range(len(cargo)):
            if object_ID in cargo[i]:
                have_product = True
                object_ID = cargo[i]
                break
        if have_product == False:
            return False, json.loads('{"error":"product not loaded"}')

        # send drop command
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                          vrep.sim_scripttype_childscript, 
                                          'dropAtPlace', [], [], 
                                          [object_ID, place_ID], 
                                          self.empty_buff, 
                                          vrep.simx_opmode_oneshot_wait))

        # check error_code
        try:
            if retInts[0] == 0:
                return False, json.loads('{"error":"'+retStrings[0]+'"}')
            elif retInts[0] == 1:
                # check if operation completed
                time.sleep(1)            
                #verify if robot has completed the operation
                ret_code, ret_msg = self.arm_move_done()
                while ret_msg["arm"] == 'True':
                    ret_code, ret_msg = self.arm_move_done()

                    time.sleep(0.5)
                    
                return True, json.loads('{"dropAt":"Done"}')
            else:
                return False, json.loads('{"error":"'+retStrings[0]+'"}')
        except ValueError:
            return False, json.loads('{"error":"return type not supported"}')
        except:
            return False, json.loads('{"error":"internal error"}')


    def get_cargo(self):
        """ 
        Return robot cargo status 
        """

        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                          vrep.sim_scripttype_childscript,
                                          "cargoInfo", [], [], [], self.empty_buff,
                                          vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 0:                
                return False, json.loads('{"error":"'+retStrings[0]+'"}')
            elif retInts[0] == 1:
                return True, json.loads('{ "cargo": [ "'+retStrings[0]+'", "'+retStrings[1]+'", "'+retStrings[2]+'" ]}')
            else: 
                return False, json.loads('{"error":"return code error"}')
        except ValueError:
            return False, json.loads('{"error":"return type not supported"}')
        except:
            return False, json.loads('{"error":"internal error"}')
            

    def get_pose(self):
        """ 
        Return robot pose (x,y,th) 
        """

        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                     vrep.sim_scripttype_childscript,
                                     "getRobotPosition", [], [], [], self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))
        tetha = retFloats[2]*180/math.pi
        return True, json.loads('{"pose":{"x":"'+str(retFloats[0])+ \
                                '","y":"'+str(retFloats[1])+'","th":"'+str(tetha)+'"}}')

    def get_scene_position(self):
        """ 
        Return robot logical position 
        """
        global position
        return True, json.loads('{"position":"'+position+'"}')

    def get_battery_level(self):
        """ 
        Return battery level
        """

        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.battery_script_ID,
                                     vrep.sim_scripttype_childscript,
                                     "getBatteryLevel", [], [], [], self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 0:
                return False, json.loads('{"error":"reading error"}')
            elif retInts[0] == 1:
                return True, json.loads('{"battery_level": '+ str(retInts[1]) +'}')
            else:
                return False, json.loads('{"error":"unexpected return code"}')
        except:
            return False, json.loads('{"error":"internal error"}')

        
    def get_battery_properties(self):
        """ 
        Return battery information 
        """

        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, self.battery_script_ID,
                                     vrep.sim_scripttype_childscript,
                                     "getBatteryProperties", [], [], [],
                                     self.empty_buff, vrep.simx_opmode_oneshot_wait))
        
        try:
            if retInts[0] == 0:
                return False, json.loads('{"error":"reading error"}')
            elif retInts[0] == 1:
                return True, json.loads('{"battery":{"working_time":"'\
                                        +retStrings[0]+'","time_to_recharge":"'\
                                        +retStrings[1]+'"}}')
            else:
                return False, json.loads('{"error":"unexpected return code"}')
        except:
            return False, json.loads('{"error":"internal error"}')

    def set_battery_properties(self, working_time=120, time_to_recharge=10):
        """
        Set battery information
        TODO: parse vrep return and send proper return value
        """
        ret = vrep.simxCallScriptFunction(self.client_ID, self.battery_script_ID,
                                         vrep.sim_scripttype_childscript, "setBatteryProperties", 
                                         [working_time, time_to_recharge], [], [], 
                                         self.empty_buff, vrep.simx_opmode_oneshot_wait)
       
        return True, json.loads('{"battery_properties": "True"}')


    def start_recharge(self, place_ID, target_level=100):
        """ 
        Start recharging 
        """
 
        ret, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self.client_ID, 
                                          self.interface_script_ID,
                                          vrep.sim_scripttype_childscript, "recharge",
                                          [], [], ["start", place_ID], self.empty_buff,
                                          vrep.simx_opmode_oneshot_wait)

        return True, json.loads('{"start_recharge": "True"}')

    def stop_recharge(self):
        """ 
        Stop recharging 
        """

        ret = vrep.simxCallScriptFunction(self.client_ID, self.interface_script_ID,
                                          vrep.sim_scripttype_childscript, "recharge",
                                          [], [], ["stop"], self.empty_buff,
                                          vrep.simx_opmode_oneshot_wait)

        return True, json.loads('{"stop_recharge": "True"}')
