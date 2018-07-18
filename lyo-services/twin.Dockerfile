FROM maven:3-jdk-8 as build

WORKDIR /build/dependencies
RUN git clone https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && git checkout mqtt && mvn clean install
RUN git clone https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && git checkout mqtt && mvn clean install

WORKDIR /build/app

# just to speed up the rebuild
# COPY ./lyo-webapp-parent/pom.xml ./lyo-webapp-parent/pom.xml
# COPY ./webapp-twin/pom.xml ./webapp-twin/pom.xml
# COPY ./webapp-whc/pom.xml ./webapp-whc/pom.xml
# COPY ./domain-pddl/pom.xml ./domain-pddl/pom.xml
# RUN mvn -f lyo-webapp-parent/pom.xml clean verify || true

COPY . .
RUN mvn -f lyo-webapp-parent/pom.xml clean package

FROM jetty:9.3-jre8

# WARNING DO NOT CHANGE WORKDIR or set it back to what it was before
# $JETTY_BASE must be correct before starting Jetty

COPY --from=build /build/app/webapp-twin/target/*.war /var/lib/jetty/webapps/webapp.war
COPY ./webapp-twin/jetty/config.xml /var/lib/jetty/webapps/
COPY ./webapp-twin/jetty/override-web.xml /
