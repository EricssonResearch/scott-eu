"""
StdShelf_VREP
@author Ricardo Souza
@date 21-06-2016
Description: Implements abstract class ShelfBase for VREP
"""

from middleware.sim.RemoteApiInterface import *
from middleware.sim.logistics.StdShelfBase import *
import abc
import middleware.sim.vrep
import time
import math

class StdShelf_VREP(StdShelfBase):

    """
    Init
    """
    def __init__(self, _RemoteApiInterface):
        self.client_ID = _RemoteApiInterface.get_client_ID()
        self.empty_buff = bytearray()


    """
    Add product to shelf
    @abstractmethod
    """
    def add_product(self, shelf_ID, product_type):
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, shelf_ID, 
                                     vrep.sim_scripttype_childscript,
                                     "addProduct", [], [], [product_type], 
                                     self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 0:
                return False, json.loads('{"error":"'+retStrings[0]+'"}')
            elif retInts[0] == 1:
                return True, json.loads('{"add":"'+retStrings[0]+'"}')
            else:
                return False, json.loads('{"error":"return code error"}')
        except ValueError:
            return False, json.loads('{"error":"return type not supported"}')
        except:
            return False, json.loads('{"error":"internal error"}')

    """
    Fill Shelf
    @abstractmethod
    """
    def fill(self, shelf_ID, stock=[10,10,10]):

        msg_yellow = None
        msg_red = None
        msg_green = None

        # yellow products
        for i in range(stock[0]):
            ret_code, msg = self.add_product(shelf_ID, 'productYellow')
            if not ret_code:
                msg_yellow = msg["error"]
                break
            msg_yellow = msg["add"]
            time.sleep(.4)
            
        # red products
        for i in range(stock[1]):
            ret_code, msg = self.add_product(shelf_ID, 'productRed')
            if not ret_code:
                msg_red = msg["error"]
                break
            msg_red = msg["add"]
            time.sleep(.4)

        # green products
        for i in range(stock[2]):
            ret_code, msg = self.add_product(shelf_ID, 'productGreen')
            if not ret_code:
                msg_green = msg["error"]
                break
            msg_green = msg["add"]
            time.sleep(.4)

        return True, json.loads('{"fill":["'+msg_yellow+'","'+msg_red+'","'+msg_green+'"]}')


    """
    Get Pickables
    @abstractmethod
    """
    def get_pickable_list(self, shelf_ID):
        error_code, retInts, retFloats, retStrings, retBuff = \
        (vrep.simxCallScriptFunction(self.client_ID, shelf_ID, 
                                     vrep.sim_scripttype_childscript,
                                     "getListOfPickableProducts", [], [], [], 
                                     self.empty_buff,
                                     vrep.simx_opmode_oneshot_wait))
        try:
            if retInts[0] == 1:
                return True, json.loads('{"pickable":["'+retStrings[0]+'","'+
                                        retStrings[1]+'","'+retStrings[2]+'"]}')
            elif retInts[0] == 0:
                return False, json.loads('{"error":"'+retStrings+'"}')
            else:
                return False, json.loads('{"error":"execution error"}')
        except ValueError:
            return False, json.loads('{"error":"return type not recognized"}')
        except:
            return False, json.loads('{"error":"internal error"}')

