FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install SWI-Prolog, git
RUN apt-get -yq install software-properties-common
RUN apt-add-repository ppa:swi-prolog/devel
RUN apt-get update
RUN apt-get install -yq swi-prolog git

# install ClioPatria
WORKDIR /opt
RUN git clone https://github.com/ClioPatria/ClioPatria.git
RUN mkdir /opt/OntologyServer
WORKDIR /opt/OntologyServer
RUN sh ../ClioPatria/configure
COPY users.db .

# install OSLC server
ARG PUBLIC_HOST
ARG PUBLIC_PORT
ARG BASE_URI
ARG EXPOSED_PREFIXES
WORKDIR /opt
RUN git clone https://github.com/EricssonResearch/scott-eu.git
RUN mkdir -p /opt/OntologyServer/cpack
WORKDIR /opt/OntologyServer/cpack
RUN ln -s ../../scott-eu/oslc_prolog/
WORKDIR /opt/OntologyServer
COPY settings.db .
RUN sed -i 's|%PUBLIC_HOST%|'$PUBLIC_HOST'|g' settings.db
RUN sed -i 's/%PUBLIC_PORT%/'$PUBLIC_PORT'/g' settings.db
RUN sed -i 's|%BASE_URI%|'$BASE_URI'|g' settings.db
RUN sed -i 's/%EXPOSED_PREFIXES%/'$EXPOSED_PREFIXES'/g' settings.db
RUN swipl run.pl --after_load='cpack_configure(oslc_prolog), halt'

# expose port
EXPOSE 3020

# start ontology server
WORKDIR /opt/OntologyServer
CMD ["swipl","run.pl"]
