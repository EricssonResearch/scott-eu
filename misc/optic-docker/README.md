
This is an early example of using optic with Docker. Due to the desgn of optic a 32 bit base image is used

To build: docker build -t optic . (in the same directory as the Dockerfile)
To run: docker run -ti optic work/gripper-domain-1.pddl work/gripper-problem-1.pddl
