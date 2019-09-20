FROM ubuntu:16.04 as build

RUN apt-get update
RUN apt-get -yq upgrade

# install gcc, make, flex and bison
# Andrew Coles of Strathclyde University tested Metric FF v2.1 with
#     flex 2.5.34 and 2.5.35, as well as bison 2.3 and 2.4.1.
RUN apt-get install -yq make gcc wget flex bison

# download ff-metric from source
WORKDIR /opt
RUN wget --quiet https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.1.tgz

# uncompress Metric-FF
RUN tar xfz Metric-FF-v2.1.tgz

# compile Metric-FF
WORKDIR /opt/Metric-FF-v2.1
RUN make

FROM ubuntu:16.04

COPY --from=build /opt/Metric-FF-v2.1/ff /usr/local/bin/

ENTRYPOINT ["ff"]
