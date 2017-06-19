FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install SWI-Prolog, git
RUN apt-get -yq install software-properties-common
RUN apt-add-repository ppa:swi-prolog/devel
RUN apt-get update
RUN apt-get install -yq swi-prolog git screen

# install ClioPatria
WORKDIR /opt
RUN git clone https://github.com/ClioPatria/ClioPatria.git
RUN mkdir /opt/OntologyServer
WORKDIR /opt/OntologyServer
RUN sh ../ClioPatria/configure

# expose port
EXPOSE 3020

# start ontology server
WORKDIR /opt/OntologyServer
CMD swipl run.pl
