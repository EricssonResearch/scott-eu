# need git
FROM maven:3-jdk-8

# COPY m2_temp/* /root/.m2/repository/
RUN echo "2019-04-22 23:33 CEST"
RUN git clone --depth=1 --branch=scott-master https://github.com/kth-mda/lyo.core.git && cd lyo.core/oslc4j-core-build && mvn clean install -Dmaven.test.skip=true -B -q
RUN git clone --depth=1 --branch=b-new-cleanup https://github.com/eclipse/lyo.client.git && cd lyo.client/oslc4j-client && mvn clean install -Dmaven.test.skip=true -B -q
RUN git clone --depth=1 --branch=master https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && mvn clean install -Dmaven.test.skip=true -B -q
RUN git clone --depth=1 --branch=mqtt-oslc4j_client https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && mvn clean install -Dmaven.test.skip=true -B -q
