import os, sys, json, re
import cStringIO, StringIO, io
from flask import Flask, jsonify, abort, request, make_response
import subprocess
import logging

#planner path - application
ff_path='/opt/Metric-FF-v2.0/ff'

#upload path
UPLOAD_FOLDER = '/uploads'
# host name and port
# HOST = 127.0.0.1
# PORT = 5000

# service route
# '/planner'
app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Helper functions
def process_ffplan(plan):
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
	print "Final plan", finalPlan
	return finalPlan

# Service Entry points
@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found'}), 404)

@app.route('/planner/test', methods=['GET'])
def test_service():
	return make_response(jsonify({'Tested ok': 'Planner Service found'}), 200)

@app.route('/planner/upload_domain', methods=['GET', 'POST'])
def upload_domain():
	if request.method == 'POST':
		# check if the post request has the file part
		if 'file' not in request.files:
			flash('No file part')
			return redirect(request.url)
		f = request.files['file']
		print "file to be uploaded", f.filename
		#check if it is a pddl domain file
		
		f.save(f.filename)
		#f.save(os.path.join(app.config['UPLOAD_FOLDER'], f.filename))
		#f.save(secure_filename(f.filename))
		return make_response(jsonify({'message' : 'file uploaded successfully'}),200)
	else:
		return make_response(jsonify({'error': 'Need to POST a domain file'}), 404)

@app.route('/planner/upload_problem', methods=['GET', 'POST'])
def upload_problem():
	if request.method == 'POST':
		# check if the post request has the file part
		if 'file' not in request.files:
			flash('No file part')
			return redirect(request.url)
		f = request.files['file']
		print "file to be uploaded", f.filename
		#check if it is a pddl problem file
		
		f.save(f.filename)
		#f.save(os.path.join(app.config['UPLOAD_FOLDER'], f.filename))
		#f.save(secure_filename(f.filename))
		return make_response(jsonify({'message' : 'file uploaded successfully'}),200)
	else:
		return make_response(jsonify({'error': 'Need to POST a domain file'}), 404)

@app.route('/planner/generate_plan', methods=['GET', 'POST'])
def generate_plan():
	if request.method == 'GET':
		if not request.json:
			abort(400)
		plannerDetail = request.json
		print plannerDetail
		
		planner     = plannerDetail["planner"]
		domainFile 	= plannerDetail["domain"]
		problemFile = plannerDetail["problem"]
		print planner, domainFile, problemFile
		
		if plannerDetail["planner"] == "FF": # call FF planner
			outputPlan = subprocess.check_output([ff_path,'-p', './', '-o', domainFile,'-f', problemFile])
			plan = outputPlan.decode('utf-8') 
			print plan
		else:
			return make_response(jsonify({'error': 'Unknown Planner Specification'}), 404)
			
		print "Process the output of FF planner"
		processedPlan = process_ffplan(plan)
		return jsonify(processedPlan), 200
	else:
		return make_response(jsonify({'error': 'Need to call with GET $HOST/planner/generate_plan'}), 404)
		
if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
