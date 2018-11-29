import cherrypy
import time
import logging
import random
import simplejson as json
from collections import namedtuple
from sim_data import *
import sys
import traceback

log = logging.getLogger("estimator")
log.setLevel(logging.INFO)
ch = logging.StreamHandler()
log.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(asctime)s - %(message)s')
ch.setFormatter(formatter)
log.addHandler(ch)


class CalculateEstimative(object):

	def __init__(self):
		self._index = {}
		for i in range(len(poi_list)):
			self._index[poi_list[i]] = i

		self.agents = None

	def _get_total_move_cost(self, moves):

		current = random.choice(wp_list)
		total_cost = 0

		for item in moves:
			previous_wp = waypoint_dict[current]
			next_wp = waypoint_dict[item["par"]["target"]]
			log.debug("%s -> %s"%(previous_wp, next_wp))
			temp_cost = cost_matrix[self._index[previous_wp]][
				self._index[next_wp]]
			total_cost += float(temp_cost)
			current = item["par"]["target"]

		return total_cost

	def _get_total_move_cost_vector(self, moves):

		current = random.choice(wp_list)
		total_cost = 0

		for item in moves:
			previous_wp = waypoint_dict[current]
			next_wp = waypoint_dict[item[2]]
			log.debug("%s -> %s"%(previous_wp, next_wp))
			temp_cost = cost_matrix[self._index[previous_wp]][
				self._index[next_wp]]
			total_cost += float(temp_cost)
			current = item[2]

		return total_cost



	def _get_total_cost(self, pjson):
		pick_count = 0
		drop_count = 0
		moves = []

		try:
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

			move_cost = self._get_total_move_cost(moves)
			pick_cost = pick_count * pick_avg_time
			drop_cost = drop_count * drop_avg_time
			total_cost = move_cost + pick_cost + drop_cost
			return total_cost
		except:
			traceback.print_exc(file=sys.stdout)
			log.error("Error while parsing plan")
			return -1

	def _get_total_cost_vector(self, pjson):
		pick_count = 0
		drop_count = 0
		moves = []

		try:
			for item in pjson["plan"]:
				op = item[0]
				if op == "move":
					moves.append(item)
				elif op == "pick":
					pick_count += 1
				elif op == "drop":
					drop_count += 1
				else:
					return -1

			move_cost = self._get_total_move_cost_vector(moves)
			pick_cost = pick_count * pick_avg_time
			drop_cost = drop_count * drop_avg_time
			total_cost = move_cost + pick_cost + drop_cost
			return total_cost
		except:
			traceback.print_exc(file=sys.stdout)
			log.error("Error while parsing plan")
			return -1



	def evaluate_mission(self, pjson):
		cost = self._get_total_cost_vector(pjson)
		resp = '{"id":"'+ pjson["id"]+'",'
		if cost == -1:
			resp+=('"feasibility": False,')
			resp+=('"reason": 2,')
			resp+=('"safety_level":'+ str(random.uniform(0,1)) + ',')
			resp+=('"completion-time":'+ str(cost))
		else:
			resp+=('"feasibility": True,')
			resp+=('"reason": 0,')
			resp+=('"safety_level":'+ str(random.uniform(0,1)) + ',')
			resp+=('"completion-time":'+ str(cost))
			resp+= '}'

		return resp


	@cherrypy.expose
	@cherrypy.tools.json_out()
	@cherrypy.tools.json_in()
	def estimator(self):
		if cherrypy.request.method == "POST":
			print("******************* POST *****************")
			input_json = cherrypy.request.json
			print(input_json)
		result = self.evaluate_mission(input_json)
		return result

cherrypy.quickstart(CalculateEstimative())
