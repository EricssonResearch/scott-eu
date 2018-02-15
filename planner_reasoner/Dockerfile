FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

ARG PUBLIC_HOST
ARG PUBLIC_PORT
ARG PREFIX_PATH
ARG EXPOSED_PREFIXES

WORKDIR /opt

COPY users.db .
COPY settings.db .

RUN apt-get update && \
    apt-get -yq install git wget build-essential flex bison && \
    git clone https://github.com/ClioPatria/ClioPatria.git && \
    git clone https://github.com/EricssonResearch/oslc_prolog.git && \
    git clone https://github.com/EricssonResearch/scott-eu.git && \
    git clone https://github.com/KCL-Planning/VAL.git && \
    mkdir -p /opt/PlannerServer/cpack && \
    mv oslc_prolog /opt/PlannerServer/cpack && \
    mv scott-eu/planner_reasoner /opt/PlannerServer/cpack && \
    wget https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.0.tgz && \
    tar xfz Metric-FF-v2.0.tgz && \
    rm Metric-FF-v2.0.tgz && \
    cd /opt/Metric-FF-v2.0 && \
    make && \
    mv ff /opt/PlannerServer && \
    cd /opt/VAL && \
    make clean && make && \
    mv validate /opt/PlannerServer && \
    rm -rf /opt/scott-eu && \
    rm -rf /opt/Metric-FF-v2.0 && \
    rm -rf /opt/VAL && \
    apt-get -y remove --purge git wget build-essential flex bison && \
    apt-get -yq --no-install-recommends install software-properties-common && \
    apt-add-repository ppa:swi-prolog/devel && \
    apt-get update && \
    apt-get -yq --no-install-recommends install swi-prolog && \
    apt-get -y remove --purge software-properties-common && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/* && \
    cd /opt/PlannerServer && \
    sh ../ClioPatria/configure && \
    mv ../users.db ../settings.db . && \
    sed -i 's|%PUBLIC_HOST%|'$PUBLIC_HOST'|g' settings.db && \
    sed -i 's/%PUBLIC_PORT%/'$PUBLIC_PORT'/g' settings.db && \
    sed -i 's|%PREFIX_PATH%|'$PREFIX_PATH'|g' settings.db && \
    sed -i 's/%EXPOSED_PREFIXES%/'$EXPOSED_PREFIXES'/g' settings.db && \
    swipl run.pl --after_load='cpack_configure(oslc_prolog), cpack_configure(planner_reasoner), halt'

WORKDIR /opt/PlannerServer

EXPOSE 3020

CMD ["swipl","run.pl"]
