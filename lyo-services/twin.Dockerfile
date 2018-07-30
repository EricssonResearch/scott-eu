FROM maven:3-jdk-8 as build

WORKDIR /build/dependencies
# TODO clone it beforehand just to trigger a pull and rebuild here
RUN git clone --depth=1 --branch=mqtt https://github.com/eclipse/lyo.trs-client.git && cd lyo.trs-client && mvn clean verify -Dmaven.test.skip=true
RUN git clone --depth=1 --branch=mqtt https://github.com/eclipse/lyo.trs-server.git && cd lyo.trs-server && mvn clean verify -Dmaven.test.skip=true

# just to speed up the rebuild
# SNAPSHOT versions will be updated at the later stage anyway
WORKDIR /build/app
COPY ./lyo-webapp-parent/pom.xml ./lyo-webapp-parent/pom.xml
COPY ./webapp-twin/pom.xml ./webapp-twin/pom.xml
COPY ./webapp-whc/pom.xml ./webapp-whc/pom.xml
COPY ./domain-pddl/pom.xml ./domain-pddl/pom.xml
RUN mvn -f lyo-webapp-parent/pom.xml clean verify || true

WORKDIR /build/app
COPY . .

WORKDIR /build/dependencies
RUN cd lyo.trs-client && git pull && mvn clean install -Dmaven.test.skip=true
RUN cd lyo.trs-server && git pull && mvn clean install -Dmaven.test.skip=true

WORKDIR /build/app
RUN mvn -f lyo-webapp-parent/pom.xml clean package

# 9.4 might have to wait till JDK9
# https://github.com/eclipse/jetty.project/issues/2412
# FROM jetty:9.3-jre8
FROM jetty:9.3-jre8-alpine

# WARNING DO NOT CHANGE WORKDIR or set it back to what it was before
# $JETTY_BASE must be correct before starting Jetty

# TODO just try putting the WAR under /var/lib/jetty/webapps/ROOT.war and
# remove the dummy config anyway
COPY --from=build /build/app/webapp-twin/target/*.war /webapps/webapp.war
COPY ./webapp-twin/jetty/config.xml /var/lib/jetty/webapps/
# COPY ./webapp-twin/jetty/override-web.xml /
