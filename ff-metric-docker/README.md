
This is an early example of using ff-metric with Docker

To build: docker build -t planner . (in the same directory as the Dockerfile)
To run: docker run -ti planner -o work/gripper-domain-1.pddl -f work/gripper-problem-1.pddl

Limitations:

1) No support for parellelism (limitation of the planner)
2) domain file and problem files need to be copied directly into the image, 
therefore they need to be known to the Dockerfile (proposed resolution: create a 
shared volume between container and docker image and copy domain file and 
problem file there)
