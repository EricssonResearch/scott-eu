# Estimator

Estimator is a service which provides feedback to warehouse controller on the 
feasiblity of a plan and its safety level. In the target system, estimation will
be done based on simulation of the warehouse and the robots. Current code carries
out the estimation based on static data and is purely to check the interfaces between
the warehouse controller and the estimator.

## REST API

The estimator service exposes an HTTP POST interface through which a json file 
containing the plan is uploaded. The format must look like:
```
{
    "id": "xyz",
    "init": "initURI",
    "goal": "finalURI",
    "plan":[
	["move", "youBot#0", "Waypoint_SH#1"],
	["pick", "youBot#0", "Shelf#1", "productRed"],
	["pick", "youBot#0", "Shelf#1", "productGreen"],
	["move", "youBot#0", "Waypoint_CB"],
	["drop", "youBot#0", "ConveyorBelt", "productRed"],
	["move", "youBot#0", "Waypoint_CB#0"],
	["drop", "youBot#0", "ConveyorBelt#0", "productGreen"],
	["move", "youBot#1", "Waypoint_SH#0"],
	["pick", "youBot#1", "Shelf#0", "productYellow"],
	["move", "youBot#1", "Waypoint_CB#0"],
	["drop", "youBot#1", "ConveyorBelt#0", "productYellow"],
	["move", "youBot#1", "Waypoint_CB#1"],
	["drop", "youBot#1", "ConveyorBelt#1", "productGreen"]
	]
}

```
In the file, **initURI** is the URI for the current state of the warehouse, **finalURI**
is the URI for the final state derived from a given mission.

The output returned to warehouse controller is a JSON object:
{
	"id":"xyz",
	"feasibility": True,
	"reason": 0,
	"safety_level":0.08263125425916351,
	"completion-time":269.82189999999997
}

The "id" refers to the id of the input JSON object. The "feasibility" tells
if the plan could be carried out successfully by the robots. If it was "false"
the "reason" attribute gives the reason for infeasibility: 1- unsafe, 2 - too-long The attribute 
"safety-level" gives a value between 0 (unsafe) and 1 (safe). Values closer to
1 are "safer". The estimated plan completion time is given through the attribute
"completion-time".

## Example

cURL: curl –request POST http://<host>:8080/estimator --upload-file example_plan.json -H “Content-Type: application/json”

## Dependencies
Use python3 and cherrypy to run the server. Further dependencies are in requirements.txt.
