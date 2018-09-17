FROM ubuntu:16.04 as build

WORKDIR /tmp

RUN mkdir -p /opt/PlannerServer && apt-get update && \
    apt-get -yq install git wget build-essential flex bison && \
    git clone --depth=1 https://github.com/KCL-Planning/VAL.git && \
    wget https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.0.tgz && \
    tar xfz Metric-FF-v2.0.tgz && \
    rm Metric-FF-v2.0.tgz && \
    cd /tmp/Metric-FF-v2.0 && \
    make && \
    mv ff /opt/PlannerServer && \
    cd /tmp/VAL && \
    make clean && make && \
    mv validate /opt/PlannerServer

WORKDIR /opt

RUN git clone -b 'V3.1.1' --depth 1 https://github.com/ClioPatria/ClioPatria.git && \
    git clone --depth=1 https://github.com/EricssonResearch/oslc_prolog.git && \
    mkdir -p /opt/PlannerServer/cpack && \
    mv oslc_prolog /opt/PlannerServer/cpack


# TODO use swipl image https://hub.docker.com/_/swipl/ or https://hub.docker.com/r/jrvosse/cliopatria/
FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

ARG PUBLIC_HOST=localhost
ARG PUBLIC_PORT=3020
ARG PREFIX_PATH=/
ARG EXPOSED_PREFIXES=*

WORKDIR /opt

COPY --from=build /opt/ /opt/

RUN apt-get update && \
    apt-get -yq --no-install-recommends install software-properties-common && \
    apt-add-repository ppa:swi-prolog/devel && \
    apt-get update && \
    apt-get -yq --no-install-recommends install swi-prolog

COPY . /opt/PlannerServer/cpack/planner_reasoner/

WORKDIR /opt/PlannerServer

COPY users.db .
COPY settings.db .

RUN sh ../ClioPatria/configure && \
    sed -i 's|%PUBLIC_HOST%|'$PUBLIC_HOST'|g' settings.db && \
    sed -i 's/%PUBLIC_PORT%/'$PUBLIC_PORT'/g' settings.db && \
    sed -i 's|%PREFIX_PATH%|'$PREFIX_PATH'|g' settings.db && \
    sed -i 's/%EXPOSED_PREFIXES%/'$EXPOSED_PREFIXES'/g' settings.db && \
    swipl run.pl --after_load='cpack_configure(oslc_prolog), cpack_configure(planner_reasoner), halt'


EXPOSE 3020

CMD ["swipl","run.pl"]
