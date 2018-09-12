"""
WarehouseScene

@author Ricardo Souza
@date 19-04-2016

"""

from middleware.sim.RemoteApiInterface import *
from middleware.sim.Scene import *
from middleware.sim.logistics.StdShelfBase import *
from middleware.sim.logistics.RobotBase import *
from middleware.sim.TruckVREP import *
from middleware.sim.YoubotVREP import * # required to run scene.get_robot
from decimal import *
import abc
import time
import math
import logging
import simplejson as json

log = logging.getLogger('middleware')

class SceneManager(Scene):
    """
    Interface to Warehouse simulation operations
    """

    def __init__(self, sim_address="127.0.0.1", sim_port=19997):
        """
        Constructor
        """
        self.api = RemoteApiInterface(sim_address, sim_port)

   
    def start(self):
         """
         Start simulation
         """
         return self.api.connect()


    def stop(self):
        """
        Stop simulation
        """
        return self.api.disconnect()

    
    def get_shelf(self, type_, impl):
        """
        Return Shelf object of type_
        """
        if type_ == "std_shelf":
            for s in StdShelfBase.__subclasses__():
                if impl in s.__name__:
                    break
        return s(self.api)
        
    
    def get_robot(self, type_, id_="youBot"):
        """
        Return robot object of type_
        """
        for r in RobotBase.__subclasses__():
            if type_ in r.__name__:
                break
        return r(self.api, self, id_)

    def get_truck(self):
        """
        Return Truck object
        """
        return TruckVREP(self.api)
   
    def _time_estimate(self, pos1, pos2, vel):
        """
        Calculate estimate time between 'pos1' and 'pos2' with constant velocity 'vel'
        """
        distance = math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])
        return float(format(distance/vel, '.4f'))


    def get_robots(self, type_="VREP"):
        """
        Get Robots
        """
        scene_robots = self.api.get_robots()
        robots = []
        for r in scene_robots:
            robots.append(self.get_robot(type_, r))

        return robots

    
    def get_poi_list(self):
        """
        Get Points of Interest

        return a dictionary with the POI data
        """
        pose_dict = self.api.get_points_of_interest_dict()
        points_list = sorted(list(pose_dict))
        return points_list

    def get_waypoints_list(self):
        """
        Get Waypoints

        return a dictionary with the Waypoints data
        """
        waypoints_dict = self.api._get_waypoints()
        waypoints_list = sorted(list(waypoints_dict))
        return waypoints_list


    def get_poi_matrix(self, robot_vel=0.2):
        """
        Generate POI cost Matrix

        returns a cost matrix between all POI
        """
        pose_dict = self.api.get_points_of_interest_dict()
        points_list = sorted(list(pose_dict))
        point_count = len(points_list)
        
        # generate POI matrix with estimate times
        cost_table = [ [0 for i in range(point_count)] for j in range(point_count) ]
         
        for i in range(point_count):
            for j in range(point_count):
                if i != j:
                    cost_table[i][j] = self._time_estimate(pose_dict[points_list[i]], 
                                                           pose_dict[points_list[j]], 
                                                           robot_vel)

        return cost_table


    def get_poi_pose(self):
        pose_dict = self.api.get_points_of_interest_dict()
        return pose_dict

    def parse_scene(self):
        """
        Parse Scene POI
        """
        conv_count = 0
        conv = []
        shelf_count = 0
        shelf = []
        recharge_count = 0
        recharge = []
        
        poi = self.get_poi_list()
        
        for i in range(len(poi)):
            if 'Conveyor' in poi[i]:
                conv.append(poi[i])
                conv_count = conv_count + 1
            if 'Shelf' in poi[i]:
                shelf.append(poi[i])
                shelf_count = shelf_count + 1
            if 'Recharge' in poi[i]:
                recharge.append(poi[i])
                recharge_count = recharge_count + 1

        poi_json = '{"points_of_interest":['

        #Conveyor belts
        if conv_count > 0:
            poi_json = poi_json + '{"type":"Conveyor Belt","count":' + str(conv_count) + ',"objects":['
            for i in range(len(conv)):
                poi_json = poi_json + '"' + conv[i] + '"'
                if i == conv_count - 1:
                    break
                poi_json = poi_json + ','
            poi_json = poi_json + ']}'
            if shelf_count > 0 or recharge_count > 0:
                poi_json = poi_json + ','

        if shelf_count > 0:
            poi_json = poi_json + '{"type":"Shelf","count":' + str(shelf_count) + ',"objects":['
            for i in range(len(shelf)):
                poi_json = poi_json + '"' + shelf[i] + '"'
                if i == shelf_count - 1:
                    break
                poi_json = poi_json + ','
            poi_json = poi_json + ']}'
            if recharge_count > 0:
                poi_json = poi_json + ','

        if recharge_count > 0:
            poi_json = poi_json + '{"type":"Recharge Station","count":' + str(recharge_count) + ',"objects":['
            for i in range(len(recharge)):
                poi_json = poi_json + '"' + recharge[i] + '"'
                if i == recharge_count - 1:
                    break
                poi_json = poi_json + ','
            poi_json = poi_json + ']}'

        poi_json = poi_json + ']}'
        
        return json.loads(poi_json)
        
