"""
TaskManager

@author Ricardo Souza
@date 19-04-2016
"""

from middleware.sim.RemoteApiInterface import *
from middleware.sim.SceneManager import *
from middleware.sim.TruckVREP import *
from middleware.sim.Agent import *
from middleware.control.Task import *
from middleware.control.MissionVerification import MissionVerification as mverify
from multiprocessing import Pipe, Lock
from threading import Thread
from queue import *
import time
import simplejson as json
import threading
import os
import simpy
import logging
import collections

ongoing_operations = None  # list containing IDs of pick/drop ongoing operations
verifier = None            
scene_manager = None
task_monitor = None
plan_json = None


log = logging.getLogger('middleware')

class TaskManager():
    
    """
    Class that manages the execution workflow
    """

    def __init__(self, smanager):
        """
        Constructor
        """
        global current_task
        current_task = None
        global scene_manager
        scene_manager = smanager;

        self.agents_data = []
        agents = scene_manager.get_robots()
        self.task_queues = []
        for i in range(0, len(agents)):
            self.agents_data.append(Agent(agents[i].get_id()))

        self.worker_thread_pool = []
        global verifier
        verifier = mverify(scene_manager, self.agents_data)

    def _create_internal_pipes(self):
        
        in_pipeMonMan, out_pipeMonMan = Pipe() # Pipe monitor -> manager
        in_pipeManMon, out_pipeManMon = Pipe() # Pipe manager -> monitor

        man_p = [
            {"sender":"TaskManager", "receiver":"TaskMonitor", "pipe": in_pipeManMon},
            {"sender":"TaskMonitor", "receiver":"TaskManager", "pipe": out_pipeMonMan}
        ]

        mon_p = [
            {"sender":"TaskManager", "receiver":"TaskMonitor", "pipe": out_pipeManMon},
            {"sender":"TaskMonitor", "receiver":"TaskManager", "pipe": in_pipeMonMan}
        ]

        return [man_p, mon_p]

    def get_task_monitor(self):
        return self.monitor

    def start(self, sync=False, in_pipes=None, out_pipes=None):
        """
        Starts worker threads: comm and execution
        """
        for i in range(0, len(self.agents_data)):
            workflow_t = Thread(target=_worker_thread, args=(self.agents_data[i], sync, out_pipes))
            workflow_t.daemon = False
            workflow_t.start()
            self.worker_thread_pool.append(workflow_t)

        comm_t = Thread(target=_comm_thread, args=(in_pipes, out_pipes))
        comm_t.daemon = False
        comm_t.start()
        self.worker_thread_pool.append(comm_t)        

        return True, json.loads('{"start": "True" }')


    def stop(self):
        """
        Stops execution worker thread
        """
        if self.worker_thread_pool[0].isAlive():
            ret, msg = self.clear_queue()
            if ret == True:
                for i in range(0, len(self.agents_data)):
                    self.agents_data[i].queue.put(None)
                return True, json.loads('{"stop": "True" }')
            return False, msg
        else:
            return True, json.loads('{"stop": "True" }')

    
    def append_mission(self, pjson):
        """
        Append new tasks to queue
        Receives a JSON containing a batch of tasks to be executed
        """

        # TODO create this mapping dynamicaly
        temp_robot_id_mapping = {
            "youBot#0" : "mobilerobot_youBot",
            "youBot#1" : "mobilerobot_youBot#4"            
        }
        
        global scene_manager
        _robot = scene_manager.get_robot("VREP")

        # TODO review this verification process
        #current_position = _robot.get_scene_position()
        #plan_status = verifier.evaluate_mission(pjson, current_position)
        #if plan_status[1]['check_mission'] == "False":
        #    log.info("Not enough battery to complete plan")
        #    return plan_status

        log.debug( "Adding task(s) to  plan" )
        global plan_json
        plan_json = pjson

        try:
            for i in range(len(pjson["plan"])):
                #TODO add verification to the tasks
                task = Task(pjson["plan"][i])
                for agt in self.agents_data:
                    if agt.get_id() == temp_robot_id_mapping[task.get_target()]:
                        agt.get_queue().put(task)
                        agt.append_task_to_plan(pjson["plan"][i])
        except:
            return False, json.loads('{"error" : "Error appending plan"}')
        finally:
            return True, json.loads('{"append": "True"}')


    def clear_queue(self):
        """
        Clear task queue
        """
        try:
            for i in range(0, len(self.agents_data)):
                while not self.agents_data[i].queue.empty():
                    self.agents_data[i].queue.get()
        except:
            return False, json.loads('{"error": "Error cleaning queue"}')
        finally:
            return True, json.loads('{"clear":"True"}')


    def get_current_task(self):
        """
        Return current task object
        """
        return current_task


    def get_ongoing_operations(self):
        """
        Return pending/ongoing pick-drop operation
        """
        return ongoing_operations


def _worker_thread(agent_info, sync, out_pipes):
    """ 
    Execution worker thread 
    """

    log.debug("Worker Thread Started")

    #global current_task
    global ongoing_operations
    global scene_manager

    t_queue = agent_info.get_queue()
    _robot = scene_manager.get_robot("VREP", agent_info.get_id())
    _shelf = scene_manager.get_shelf("std_shelf", "VREP")
    log.debug("Worker Thread: {}".format(agent_info.get_id()))

    if ongoing_operations == None:
        ongoing_operations = {}

    # wrappers to call from switchers
    def _do_move(params, out_pipes = None):
        """ 
        Call robot move method 
        """
        return _robot.move_to(params['target'])


    def _do_pick(params, out_pipes = None):
        """
        Call robot pick method and update open orders
        """
        # previous cargo
        r_code, cargo_before = _robot.get_cargo()
        r, j = _robot.pick_up(params['type'], params['source'])
        if r == True:
            # cargo after op
            r_code, cargo_after = _robot.get_cargo()
            # including product ID for order
            c_order_id = list(set(cargo_after['cargo'])-set(cargo_before['cargo']))

            if not c_order_id:
                log.error("Error performing cargo loading")
                r = False
            else:
                ongoing_operations[id_] = c_order_id[0]

        else:
            log.debug("Error on Pick operation")

        return r, j


    def _do_drop(params, out_pipes = None):
        """
        Call robot drop method and check/removes open orders
        """
        try:
            product = ongoing_operations[id_]
        except KeyError:
            log.debug("No order registered with ID: {}".format(id_))
            product = params['type']
            return False, json.loads('{"error":"order not found"}')

        r, j = _robot.drop_at(product, params['target'])            
        
        # informs truck about new cargo
        if (r == True):
    
            # associating conveyor belts and trucks
            load_target = {
                "ConveyorBelt"   : "Truck#0",
                "ConveyorBelt#0" : "Truck#1",
                "ConveyorBelt#1" : "Truck#2"
            }

            truck = load_target[params['target']]
            msg = {"ptype"  : params['type']}

            for item in out_pipes:
                if ((item["sender"] == "TaskManager") and
                    (item["receiver"] == truck)):
                    item["pipe"].send(msg)
                    break
            try:
                del ongoing_operations[id_]
            except KeyError:
                log.debug("No order registered with ID: {}".format(id_))

            return r, j


    # operations switcher
    operations = {
        "move" : _do_move,
        "pick" : _do_pick,
        "drop" : _do_drop,
    }

    # workflow loop
    while True:
        if t_queue.empty():
            time.sleep(2)
        else:
            
            # get current task and parse it
            agent_info.set_current_task(t_queue.get())

            # if None stop worker thread
            if agent_info.get_current_task() is None:
                log.debug( "Stopping Worker Thread" )
                break

            # task info
            op = agent_info.get_current_task().get_operation()
            par = agent_info.get_current_task().get_parameters()
            id_ = agent_info.get_current_task().get_ID()

            log.debug("{} -> Operation: {}; ID: {}; Param: {}".format(agent_info.get_id(), op, id_, par))

            # task selection and execution
            ret, rjson = operations[op](par, out_pipes)
            if sync == True:
                if ret != True:
                    log.error( "Error on task: " + op + " - " + str(rjson))
                    while not t_queue.empty():
                        t_queue.get()
            # task completed
            t_queue.task_done()



def get_estimate(element_id):
    """
    Send estimatives to TruckManager
    """
    est = None
    est = verifier.get_time_to_completion(element_id)
    log.debug("{}: Estimate time to load: {}".format(element_id,est))
    msg = {
        "event": "expected_loading_time", 
        "sender": "TaskManager", 
        "receiver": "TruckManager", 
        "expected_loading_time": est, 
        "id": element_id
    }
    return msg


def _comm_thread(_in_pipes, _out_pipes):
    """ 
    Communication Thread 
    """
    global scene_manager
    truck = scene_manager.get_truck()
    truck_pipes = []
    
    monitor_pipe_lock = Lock()
    truckm_input_pipe = None
    truckm_output_pipe = None

    optimizer_pipes = []
    retailer_pipes = []

    # get truck pipes
    for item in [pipe for pipe in _in_pipes if "Truck" in pipe["sender"]]:
        if item['receiver'] == 'TaskManager':
            truck_pipes.append(item)

    # get optimizer pipes
    for item in [pipe for pipe in _in_pipes if "OptimizerManager" in pipe["sender"]]:
        if item['receiver'] == 'TaskManager':
            optimizer_pipes.append(item)

    # get retailers pipes
    for item in [pipe for pipe in _in_pipes if "RetailerManager" in pipe["sender"]]:
        if item['receiver'] == 'TaskManager':
            retailer_pipes.append(item)

    # get truck manager pipe
    for item in [pipe for pipe in _in_pipes if "TruckManager" in pipe["sender"]]:
        if item ['receiver'] == 'TaskManager':
            truckm_input_pipe = item["pipe"]
    
    #get monitor pipe
    for item in [pipe for pipe in _out_pipes if "TruckManager" in pipe["receiver"]]:
        if item ['sender'] == 'TaskManager':
            truckm_output_pipe = item["pipe"]


    while True:

        #truck messages
        for item in truck_pipes:
            if item["pipe"].poll():
                msg = item["pipe"].recv()
                if 'departure' in msg["event"]:
                    truck.departure(item['sender'])
                elif 'arrival' in msg["event"]:
                    truckm_output_pipe.send(get_estimate(item['sender']))
                    truck.arrival(item['sender'])
                else:
                    log.debug("Unknown message")

        '''
        #recv optimizer messages
        for item in optimizer_pipes:
            if item["pipe"].poll():
                msg = item["pipe"].recv()
                #parse msg
        
        #recv retailer messages
        for item in retailer_pipes:
            if item["pipe"].poll():
                msg = item["pipe"].recv()
                #parse msg
        '''
        time.sleep(0.5)
