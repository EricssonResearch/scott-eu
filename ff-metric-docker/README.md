Usage
------
Run the planner service by:
1) docker build -t ff-metric .
2) docker run -it -p:5000:5000 ff-metric

HOST=localhost
PORT:5000

Service entry-points and example test files using Curl
--------------------
Test : GET HOST:port/planner/test
Example : curltest-test

upload a domain file : POST HOST:port/planner/upload_domain
Example : curltest-upload-domain

upload a problem file : POST HOST:port/planner/upload_problem
Example : curltest-upload-problem

generate a plan : GET HOST:port/planner/generate_plan
Example : curltest-plan : supplies a json file containing planner, domain and problem
specification
Example : curltest-jsonplan : supplies a json string containing planner, domain and problem

To be done
-----------
1. mission2plan.py file to be modified to make the call to the planner service
2. include OPTIC and the plan extraction from OPTIC output.
3. Consider code modifications to FF and OPTIC to directly generate the processed plan files
4. Supply options to FF and OPTIC through the JSON input

Limitations
-----------
Planner does not support parallelism
