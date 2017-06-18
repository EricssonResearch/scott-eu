
import os, sys, json, re
import cStringIO, StringIO, io
from flask import Flask, jsonify, abort, request, make_response
import subprocess
import logging

#planner path
ff_path='/opt/work/Metric-FF-v2.0/ff'

app = Flask(__name__)

@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found'}), 404)

@app.route('/ff-metric/api/v1.0/test', methods=['GET'])
def test_service():
	return make_response(jsonify({'Tested ok': 'Workflow Gen Service found'}), 200)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
