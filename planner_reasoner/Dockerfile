FROM ubuntu:16.04 as build

WORKDIR /tmp

ARG CHECKSUM=146d456a93da57f2c2a0c51b6a4b8711be9626e3d07a2878a6d8edc61d01857b

RUN apt-get update && \
    apt-get -yq install git wget build-essential flex bison

# wget https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.0.tgz && \
# Uni Saarland was timing out from time to time and failing builds
# VAL removed Makefile support without any release or tag, pinned the last make-based hash
RUN mkdir -p /opt/PlannerServer && \
    wget https://aide.md.kth.se/uploads/Metric-FF-v2.0.tgz && \
    echo "$CHECKSUM Metric-FF-v2.0.tgz" | sha256sum -c - || exit 1 && \
    tar xfz Metric-FF-v2.0.tgz && \
    rm Metric-FF-v2.0.tgz && \
    cd /tmp/Metric-FF-v2.0 && \
    make && \
    mv ff /opt/PlannerServer

RUN git clone https://github.com/KCL-Planning/VAL.git && \
    cd /tmp/VAL && git reset --hard a5565396007eee73ac36527fbf904142b3077c74 && \
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
