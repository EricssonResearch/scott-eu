"""
MissionVerification

@author Ricardo Souza
@date 15-08-2016
"""

from middleware.sim.RemoteApiInterface import *
from middleware.sim.SceneManager import *
from collections import namedtuple
import time
import simplejson as json
import logging
import traceback

log = logging.getLogger('middleware')

waypoint_dict = {
    "Waypoint_SH"   : "Shelf",
    "Waypoint_SH#0" : "Shelf#0",
    "Waypoint_SH#1" : "Shelf#1",
    "Waypoint_CB"   : "ConveyorBelt",
    "Waypoint_CB#0" : "ConveyorBelt#0",
    "Waypoint_CB#1" : "ConveyorBelt#1",
    "Waypoint_RS"   : "RechargeStation",
    "Waypoint_RS#0" : "RechargeStation#0"
}

target_dict = {
    "Truck#0" : "ConveyorBelt",
    "Truck#1" : "ConveyorBelt#0",
    "Truck#2" : "ConveyorBelt#1"
}

AgentData = namedtuple("AgentData", ["data", "instance"])

class MissionVerification():
    """
    Verification if mission is valid
    """

    def __init__(self, sim, agents):
        """
        Constructor
        """
        self.scene = sim
        self.matrix = self.scene.get_poi_matrix()
        poi = self.scene.get_poi_list()
        self._index = {}
        for i in range(len(poi)):
            self._index[poi[i]] = i

        self.pick_avg_time = 24 # seconds  TODO: get the value from sim and update constantely
        self.drop_avg_time = 20 # seconds
        self.agents = []

        for agt in agents:
            for robot in self.scene.get_robots("VREP"):
                if robot.get_id() == agt.get_id():
                    self.agents.append(AgentData(agt, robot))

    

    def _get_total_move_cost(self, move_list, cposition, cpose=None):
        """
        Calculate total cost of move operations
        """
        current = cposition[1]['position']
        total_cost = 0

        if cpose is None:
            self.first = False
        else:
            self.first = True

        for item in move_list:
            if self.first is True:
                self.first = False
                poi_pose = self.scene.get_poi_pose()
                next_pose = poi_pose[waypoint_dict[item["par"]["target"]]]
                temp_cost = math.hypot(next_pose[0]-float(cpose[1]['pose']['x']), next_pose[1]-float(cpose[1]['pose']['y']))/0.2
                total_cost += float(temp_cost)
            else:
                previous_wp = waypoint_dict[current]
                next_wp = waypoint_dict[item["par"]["target"]]
                log.debug("%s -> %s"%(previous_wp, next_wp))
                temp_cost = self.matrix[self._index[previous_wp]][self._index[next_wp]]
                total_cost += float(temp_cost)
            current = item["par"]["target"]
        return total_cost



    def _get_remaining_battery(self, robot):
        """
        Calculate robot remaning battery time
        """
        batt_setup = float(robot.get_battery_properties()[1]['battery']['working_time'])
        batt_level = float(robot.get_battery_level()[1]['battery_level'])

        return batt_setup*batt_level/100



    def _get_total_cost(self, pjson, cpos):
        """
        Total cost of mission
        """
        pick_count = 0
        drop_count = 0
        moves = []

        try:
            #parse json
            for item in range(len(pjson["plan"])):
                op = pjson["plan"][item]["operation"]
                if op == "move":
                    moves.append(pjson["plan"][item])
                elif op == "pick":
                    pick_count += 1
                elif op == "drop":
                    drop_count += 1
                else:
                    return -1

            move_cost = self._get_total_move_cost(moves, cpos) #self.robot.get_scene_position())
            pick_cost = pick_count * self.pick_avg_time
            drop_cost = drop_count * self.drop_avg_time
            total_cost = move_cost + pick_cost + drop_cost
            return total_cost
        except:
            traceback.print_exc(file=sys.stdout)
            log.error("Error while parsing plan")
            return -1



    def evaluate_mission(self, pjson, cpos):
        """
        Check if plan can be executed
        """
        log.debug("Checking plan")
    
        robot_remaning_battery = self._get_remaining_battery()
        total_cost = self._get_total_cost(pjson, cpos)
        
        if total_cost == -1:
            return False, json.loads('{"error": "json parsing error"}')
        
            log.debug("Total cost: %s - Battery: %s"%(total_cost, robot_remaning_battery))
            
        if ((robot_remaning_battery - total_cost) <= 0):
            return True, json.loads('{"check_mission": "False"}')
            
        log.debug("Valid plan")
        return True, json.loads('{"check_mission": "True"}')


    def _get_target_last_operation(self, pjson, target):
        """
        Get last operation related to target
        """
        for item in range(len(pjson["plan"])):
            try:
                if pjson["plan"][item]["par"]["target"] == target_dict[target]:
                    return pjson["plan"][item]["op_id"]
            except:
                continue


#    def _get_robot_instance(self, agent_id):
#        """
#        Get robot instance for a specific agent
#        """
#        for robot in self.robots:
#            if robot.get_id() == agent_id:
#                return robot


    def _get_agent_estimative(self, agent, target=None):
        """
        Get agent's plan time to completion
        """
        pick_count = 0
        drop_count = 0
        moves = []

        pjson = agent.data.get_plan()

        target_op = None
        if target is not None:
            target_op = self._get_target_last_operation(pjson, target)
        
        if target_op is None:
            return 0

        try:
            #get remaining tasks
            c_task = agent.data.get_current_task()
            if c_task is None:
                self.past_tasks = False
            else:
                self.past_tasks = True

            for item in range(len(pjson["plan"])):
                if self.past_tasks is True:
                    if pjson["plan"][item]["op_id"] == c_task.get_operation_id():
                        self.past_tasks = False
                else:
                    op = pjson["plan"][item]["operation"]
                    if op == "move":
                        moves.append(pjson["plan"][item])
                    elif op == "pick":
                        pick_count += 1
                    elif op == "drop":
                        drop_count += 1
                    else:
                        return -1
                if pjson["plan"][item]["op_id"] == target_op:
                    break
                    

            move_cost = self._get_total_move_cost(moves, agent.instance.get_scene_position(), agent.instance.get_pose())
            pick_cost = pick_count * self.pick_avg_time
            drop_cost = drop_count * self.drop_avg_time
            total_cost = move_cost + pick_cost + drop_cost
            return total_cost
        except:
            traceback.print_exc(file=sys.stdout)
            log.error("Time To Complete: Error while parsing plan")
            return -1
        

    def get_time_to_completion(self, target=None):
        """
        Total cost of mission
        """
        est = -1
        for agent in self.agents:
            t_est = self._get_agent_estimative(agent, target)
            if (t_est > est):
                est = t_est

        return est
