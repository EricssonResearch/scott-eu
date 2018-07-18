FROM maven:3-jdk-8 as build

WORKDIR /build/dependencies
RUN git clone https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && git checkout mqtt && mvn clean install
RUN git clone https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && git checkout mqtt && mvn clean install

WORKDIR /build/app
COPY . .
RUN mvn -f lyo-webapp-parent/pom.xml clean package

FROM jetty:9.3-jre8

COPY --from=build /build/app/webapp-whc/target/*.war /var/lib/jetty/webapps//webapp.war
COPY ./webapp-whc/jetty/config.xml /var/lib/jetty/webapps/
COPY ./webapp-whc/jetty/override-web.xml /
