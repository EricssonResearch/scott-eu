# -*- coding: utf-8 -*-
#!flask/bin/python
from flask import Flask, jsonify

app = Flask(__name__)

waypoint = [
    {'id' : 'xyz',
    'time' : 'time', # time at which snapshot is recovered from KB
    'waypointList' : [['a1', 'b1'], ['a2', 'b2']], # all points are reachable from each other so there is no connectivity information need to be maintained

    'bayList': {
        'bay1': {'location' : ['a1', 'b1'], 'capacity' : 5},
        'bay2': {'location' : ['a2', 'b2'], 'capacity' : 1}
    },

    'shelfList' : {
        'shelf1': {'location' : ['a1', 'b1'], 'capacity' : 5},
        'shelf2': {'location' : ['a2', 'b2'], 'capacity' : 1}
    },

    'objectList' : {
        'o1' : 'shelf1',
        'o2' : 'shelf2',
        'o3': 'bay1'
    },

    'robotList' : {
        'robot1' : {'location':['a1', 'b1'], 'charge_level':'70pct', 'capacity':5},
        'robot1' : {'location':['a2', 'b2'], 'charge_level':'30pct', 'capacity':2}
    }}]



@app.route('/kb/api/v1.0/waypoint', methods=['GET'])
def get_tasks():
    return jsonify({'waypoint': waypoint})

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
