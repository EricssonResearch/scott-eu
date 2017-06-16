FROM ubuntu:16.10
LABEL maintainer "konstantinos.vandikas@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install gcc, make, flex and bison
RUN apt-get install -yq make gcc wget flex bison

# download ff-metric from source
WORKDIR /opt
RUN wget --quiet https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.0.tgz

# uncompress Metric-FF
RUN tar xfz Metric-FF-v2.0.tgz

# compile Metric-FF
WORKDIR /opt/Metric-FF-v2.0
RUN make

# copy problem/domain files
#RUN mkdir work
#COPY gripper-problem-1.pddl work
#COPY gripper-domain-1.pddl work

# set entrypoint
ENTRYPOINT [ "/opt/Metric-FF-v2.0/ff" ]
