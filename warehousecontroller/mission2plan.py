import os, sys, json, re
import cStringIO, StringIO, io
from flask import Flask, jsonify, abort, request, make_response
import subprocess
import logging

#planner path - application
optic_path='/home/swarup/Documents/optic/debug/optic/optic-clp'
ff_path='/home/swarup/Documents/Metric-FF-v2.0/ff'

# use end point instead of path
optic_end_point = "http://tools_ff_metric_1:5000"

app = Flask(__name__)

@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found'}), 404)

@app.route('/warehouse/api/v1.0/test', methods=['GET'])
def test_service():
	return make_response(jsonify({'Tested ok': 'Workflow Gen Service found'}), 200)

def extractGoals(inp):
	return inp['goal']

def extractHints(inp):
	return [inp['responseTimeout'], inp['missionDeadline']]

def queryKB(spec, obj, level):
	tempKBJsonFile = 'kb.json'
	with open(tempKBJsonFile) as data:   	
		kb = (json.load(data))
	#print kb
	return kb

def constructPddlProblem(entity, level, whState, missionGoal, missionHints):
	pddlProblemHeaderString = "(define (problem " + entity + "-" + level + ")\n" \
					"\t (:domain warehouse-domain)\n"
	objectString = "\n\t(:objects"
	initString = "\n\t(:init"
	goalString = "\n\t(:goal\n\t\t (and"
	metricString = "\n\t(:metric minimize (total-time))"
	
	# Robot Current State
	robotList = whState["Robot"]
	for robot in robotList.keys():
		robotDetails = robotList[robot]
		objectString = objectString + "\n\t\t" + robot + " - " + "Robot"
		initString = initString + "\n\t\t" + "(is-at " + robot + " " + robotDetails["location"] + ")"
		initString = initString + "\n\t\t" + "(= (charge-level " + robot + ") " + robotDetails["charge-level"] + ")"
		initString = initString + "\n\t\t" + "(= (capacity " + robot + ") " + robotDetails["capacity"] + ")"
		initString = initString + "\n\t\t" + "(= (max-charge " + robot + ") " + robotDetails["max-charge"] + ")"
		initString = initString + "\n\t\t" + "(= (high-charge " + robot + ") " + robotDetails["high-charge"] + ")"
		initString = initString + "\n\t\t" + "(= (low-charge " + robot + ") " + robotDetails["low-charge"] + ")"
		if robotDetails["charging-state"] == "0":
			initString = initString + "\n\t\t" + "(not (is-recharging " + robot + "))"
		else:
			initString = initString + "\n\t\t" + "(is-recharging " + robot + ")"
	
	#Place current situation
	placeList = whState["Place"]
	for place in placeList.keys():
		placeDetails = placeList[place]
		objectString = objectString + "\n\t\t" + place + " - " + "Place"
		initString = initString + "\n\t\t" + "(situated-at " + place + " " + placeDetails["location"] + ")"
	
	#Objects
	objectList = whState["Object"]
	for object in objectList.keys():
		objectDetails = objectList[object]
		objectString = objectString + "\n\t\t" + object + " - " + "Object"
		initString = initString + "\n\t\t" + "(is-on " + object + " " + objectDetails["location"] + ")"
	
	#Waypoints
	waypointList = whState["Waypoint"]
	print waypointList
	for wpt in waypointList.keys():
		objectString = objectString + "\n\t\t" + wpt + " - " + "Waypoint"
		for w in waypointList[wpt]["can-move"]:
			initString = initString + "\n\t\t" + "(can-move " + wpt + " " + w + ")"
		
	#Goals : this will come from the mission
	for goal in missionGoal:
		goalString = goalString + "\n\t\t\t(is-on " + goal[0] + " " + goal[1] + ")"
	
	OutFile = open("warehouseProblemGen.pddl", 'w')
	
	OutFile.write(pddlProblemHeaderString)
	OutFile.write(objectString)
	OutFile.write("\n\t)")
	OutFile.write(initString)
	OutFile.write("\n\t)")
	OutFile.write(goalString)
	OutFile.write("\n\t\t)\n\t)\n")
	OutFile.write(metricString)
	OutFile.write("\n )")
	
	print pddlProblemHeaderString
	print objectString
	print ")\n"
	print initString
	print ")\n"
	print goalString
	print "))\n"
	print metricString
	print ")"
	
@app.route('/warehouse/api/v1.0/genplan', methods=['POST'])
def compute_mission_data():
	if not request.json:
        	abort(400)
	mission = request.json
	print mission
	
	# Extract mission and hints
	#mission = extractMission(mission)
	goals = extractGoals(mission)
	hints   = extractHints(mission)
	
	print "mission, goals, hints"
	print mission, goals, hints
	
	# get waypoint model and policies from KB
	warehouseState = queryKB("state", "warehouse", "waypoint")
	#print warehouseState
	#waypointPolicy = extractWaypointPolicy("policy", "warehouse", "waypoint")
	#p waypointPolicy
	
	# init state - needed to optimize
	#warehouseInitState = extractInit(waypointState, goals)

	pddlProblem = constructPddlProblem("warehouse", "waypoint", warehouseState, goals, hints)
	# - to be taken up later addPolicies(pddlDomain, policy)
	
	##outputPlan = subprocess.check_output([optic_path,'-E','domain.pddl','problem.pddl'])
	outputPlan = subprocess.check_output([ff_path,'-p', './', '-o', 'whdomain-2.pddl','-f','warehouseProblemGen.pddl'])
	plan = outputPlan.decode('utf-8') 
	print plan
	
	print "Process the output of FF planner"
	#extract plan, jsonify and return
	planData = (StringIO.StringIO(plan)).readlines()
	# print planData
	
	# find the 1st step pf plan matching step
	regex = re.compile("step*")
	lst = [m for l in planData for m in [regex.search(l)]]
	m1 = max(index for index, item in enumerate(lst, 0) if item is not None)
	print m1, planData[m1]
	
	regex = re.compile("plan cost*")
	lst = [m for l in planData for m in [regex.search(l)]]
	m2 = max(index for index, item in enumerate(lst, 0) if item is not None)
	print m2, planData[m2]
	
	finalPlan = []
	if (m2 > m1) :
		for i in range(m1,m2, 1):
			step = planData[i].split(':')[1]
			print step
			planStep = (step.strip()).split(' ')
			print planStep
			if (planStep[0] == 'moveToWaypoint'):
				print "changing the format of move to suit the MOO_Manager"
				del(planStep[2])
				step = " ".join(str(x) for x in planStep)
			finalPlan.append(step.strip())
			# print step[1]
	
	print "Final plan"
	print finalPlan
	return jsonify(finalPlan), 200
# def gen_workflow():
	# if not request.json:
        	# abort(400)
	# inp = request.json
	# print inp
	
	# # Extract mission and hints
	# mission = extractMission(inp)
	# goals = extractGoals(mission)
	# hints   = extractHints(mission)
	
	# # get waypoint model and policies from KB
	# waypointModel = queryKB("warehouse", "waypoint")
	# waypointState = extractWaypointState(waypointModel)
	# waypointPolicy = extractWaypointPolicy(waypointModel)
	

	# return jsonify(graph), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
