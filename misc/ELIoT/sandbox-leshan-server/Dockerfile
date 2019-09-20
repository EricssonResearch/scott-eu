FROM openjdk:8-jre-alpine

ADD target/sandbox-leshan-0.0.1-SNAPSHOT.jar /sandbox-leshan.jar

EXPOSE 5683 5684 8080

WORKDIR /
ENTRYPOINT ["java", "-jar", "sandbox-leshan.jar"]

