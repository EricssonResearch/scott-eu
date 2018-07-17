FROM maven:3-jdk-8

WORKDIR /build/dependencies
RUN git clone https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && git checkout mqtt && mvn clean install
RUN git clone https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && git checkout mqtt && mvn clean install

WORKDIR /build/app
COPY . .
RUN mvn clean package

FROM jetty:alpine

WORKDIR /var/lib/jetty/webapps/

COPY --from=0 /build/app/target/*.war ./webapp.war
COPY jetty/config.xml .
COPY jetty/override-web.xml /
