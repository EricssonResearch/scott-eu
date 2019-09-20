FROM ubuntu:18.04
LABEL maintainer "konstantinos.vandikas@ericsson.com"

# update/upgrade base system
RUN apt-get update && apt-get -yq upgrade \
  && apt-get -yq install --no-install-recommends wget unzip \
  && rm -rf /var/lib/apt/lists/*

# download emqtt
WORKDIR /opt
# amateurs!
RUN wget --quiet --no-check-certificate https://www.emqx.io/downloads/broker/v2.3.9/emqttd-ubuntu18.04-v2.3.9.zip
RUN unzip emqttd-ubuntu18.04-v2.3.9.zip

# start emqtt daemon
CMD /opt/emqttd/bin/emqttd start && tail -f /dev/null
