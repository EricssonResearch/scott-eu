I`m sending a simple estimator.py, a file with some hardcode e an example plan.
It uses cherrypy to run the server and python3.

I`m testing using 
cURL: curl –request POST http://<host>:8080/estimator --upload-file example_plan.json -H “Content-Type: application/json”
