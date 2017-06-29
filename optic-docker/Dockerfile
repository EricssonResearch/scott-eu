FROM f69m/ubuntu32:lts
LABEL maintainer "konstantinos.vandikas@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install requirements for optic planner (since we are going for the binary version not all of this is required)
RUN apt-get install -yq wget bzip2 g++ cmake coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev coinor-libosi-dev coinor-libcgl-dev libbz2-dev bison flex

# download optic 32-bit binary
WORKDIR /opt
RUN wget --quiet https://downloads.sourceforge.net/project/tsgp/OPTIC/optic-clp.tar.bz2

# uncompress optic
RUN bzip2 -d optic-clp.tar.bz2
RUN tar xf optic-clp.tar

# compile
WORKDIR /opt/optic-clp
RUN make

# copy problem/domain files
#RUN mkdir work
#COPY gripper-problem-1.pddl work
#COPY gripper-domain-1.pddl work

# set entrypoint
ENTRYPOINT [ "/opt/optic-clp/optic-clp" ]
