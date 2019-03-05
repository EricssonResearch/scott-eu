FROM maven:3-jdk-8

# COPY m2_temp/* /root/.m2/repository/

# TODO switch to master once https://github.com/eclipse/lyo.client/pull/37 is merged
# TODO delete once https://github.com/eclipse/lyo.client/issues/40 and https://github.com/EricssonResearch/scott-eu/issues/150 are fixed
RUN git clone --depth=1 --branch=jra-rename https://github.com/eclipse/lyo.client.git && cd lyo.client/oslc4j-client && mvn clean install -Dmaven.test.skip=true -B -q

RUN git clone --depth=1 --branch=master https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && mvn clean install -Dmaven.test.skip=true -B -q
RUN git clone --depth=1 --branch=mqtt-oslc4j_client https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && mvn clean install -Dmaven.test.skip=true -B -q
