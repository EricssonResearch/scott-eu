Simple estimator.py for static estimation
Dependencies : cherrypy to run the server and 
               python3.

example_plan.json gives a plan 

Test: 
cURL: curl –request POST http://<host>:8080/estimator --upload-file example_plan.json -H “Content-Type: application/json”
