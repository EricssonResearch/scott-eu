FROM scott/maven-base:latest

WORKDIR /build/app

COPY ./lyo-webapp-parent/pom.xml ./lyo-webapp-parent/pom.xml
COPY ./webapp-twin/pom.xml ./webapp-twin/pom.xml
COPY ./webapp-executor/pom.xml ./webapp-executor/pom.xml
COPY ./webapp-whc/pom.xml ./webapp-whc/pom.xml
COPY ./webapp-svc-location/pom.xml ./webapp-svc-location/pom.xml
COPY ./domain-pddl/pom.xml ./domain-pddl/pom.xml
#RUN mvn -f lyo-webapp-parent/pom.xml dependency:resolve  -B -q || true
RUN mvn -f lyo-webapp-parent/pom.xml --fail-at-end dependency:go-offline -B -q || true

COPY . .
RUN mvn -f lyo-webapp-parent/pom.xml --no-snapshot-updates -Dmaven.test.skip=true package -B -q
