FROM maven:3-jdk-8

WORKDIR /build/dependencies
RUN git clone https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && git checkout mqtt && mvn clean install
WORKDIR /build/dependencies
RUN git clone https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && git checkout mqtt-jaxrs_1.1 && mvn clean install
