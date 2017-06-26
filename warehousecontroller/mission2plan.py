import os, sys, json, re
import cStringIO, StringIO, io
from flask import Flask, jsonify, abort, request, make_response
import subprocess, requests
import logging

#planner path - application
optic_path='/home/swarup/Documents/optic/debug/optic/optic-clp'
ff_path='/home/swarup/Documents/Metric-FF-v2.0/ff'

# planner 
planner = "FF" 

#Warehouse Domain File
warehouseDomainFile = "whdomain-2.pddl"

# use end point instead of path
ff_end_point = "http://ff-metric_1:5000"
#ff_end_point = "http://127.0.0.1:5000"

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

def generatePddlProblemFile(pddlProblemFileName, entity, level, whState, missionGoal, missionHints):
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
	
	OutFile = open(pddlProblemFileName, 'w') #overwrites the earlier file
	
	OutFile.write(pddlProblemHeaderString)
	OutFile.write(objectString)
	OutFile.write("\n\t)")
	OutFile.write(initString)
	OutFile.write("\n\t)")
	OutFile.write(goalString)
	OutFile.write("\n\t\t)\n\t)\n")
	OutFile.write(metricString)
	OutFile.write("\n )")
	
	OutFile.close()
	print pddlProblemHeaderString
	print objectString
	print ")\n"
	print initString
	print ")\n"
	print goalString
	print "))\n"
	print metricString
	print ")"
	


def callPlanner(warehouseState, goals, hints):
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
	return finalPlan
	
@app.route('/warehouse/api/v1.0/genplan', methods=['POST'])
def compute_mission_data():
	if not request.json:
        	abort(400)
	mission = request.json
	
	# Extract mission and hints
	goals = extractGoals(mission)
	hints   = extractHints(mission)

	print "mission, goals, hints"
	print mission, goals, hints
	
	# get waypoint model and policies from KB
	warehouseState = queryKB("state", "warehouse", "waypoint")
	#print warehouseState
	#waypointPolicy = extractWaypointPolicy("policy", "warehouse", "waypoint")
	#p waypointPolicy
	
	#Generate problem file which is in the string warehouseProblemFile
	warehouseProblemFile = "warehouseProblemGen.pddl"
	generatePddlProblemFile(warehouseProblemFile, "warehouse", "waypoint", warehouseState, goals, hints)
	
	#Get the domain file whose name is a string in warehouseDomainFile
	#warehouseDomainFile = 

	res = requests.post(ff_end_point+"/planner/upload_domain", files={'file': open(warehouseDomainFile, 'rb')})
	print res.text, warehouseDomainFile
	res = requests.post(ff_end_point+"/planner/upload_problem", files={'file': open(warehouseProblemFile, 'rb')})
	print res.text, warehouseProblemFile
	
	payload = {"planner" : planner, "domain" : warehouseDomainFile, "problem" : warehouseProblemFile}
	res = requests.get(ff_end_point+"/planner/generate_plan", json=payload)
	print res.json()
	
	return jsonify(res.json()), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
