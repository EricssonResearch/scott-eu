"""
Monitor

@author Ricardo Souza
@date 14-11-2016
"""

from middleware.sim.SceneManager import *
from middleware.sim.RemoteApiInterface import *
from multiprocessing import Lock

import time
import simplejson as json
import logging
import traceback

log = logging.getLogger('middleware')

class Monitor():
    """
    Interface to the Task execution
    """

    def __init__(self, sim, cpipes):
        """
        Constructor
        """
        self._scene = sim
        self.comm_out = [item for item in cpipes if "TaskMonitor" in  item["sender"]][0]["pipe"]
        self.comm_in = [item for item in cpipes if "TaskMonitor" in item["receiver"]][0]["pipe"]

        log.debug("Monitor Started")


    def get_total_estimative(self, truckID):
        """
        Get estimative for truck load completion
        """
        msg = "total_estimative:"+truckID
        self.comm_out.send(msg)
        #log.debug("Get_Total_Estimative: " + str(msg))
        timeout = time.time() + 10
        while time.time() <= timeout:
            if self.comm_in.poll():
                ret_msg = self.comm_in.recv()
                return ret_msg
            time.sleep(1)
        
        return -1
        

    def get_partial_estimative(self, truckID):
        """
        Parse plan for specific truck
        """
        msg = "partial_estimative:"+truckID
        self.comm_out.send(msg)
        #log.debug("Get_Partial_Estimative: " + str(msg))
        timeout = time.time() + 10
        while time.time() <= timeout:
            if self.comm_in.poll():
                #log.debug(self.comm_in.recv())
                #break
                ret_msg = self.comm_in.recv()
                return ret_msg
            time.sleep(1)
        
        return -1
