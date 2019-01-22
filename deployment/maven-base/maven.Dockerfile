FROM maven:3-jdk-8

COPY m2_temp/* /root/.m2/repository/

RUN git clone --depth=1 --branch=mqtt-2019 https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && mvn clean install -Dmaven.test.skip=true -B -q
RUN git clone --depth=1 --branch=b-newChangeEvent-2ndtry https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && mvn clean install -Dmaven.test.skip=true -B -q
