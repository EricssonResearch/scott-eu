FROM scott/sandbox-base:latest as build

FROM jetty:9.4-alpine

COPY --from=build /build/app/webapp-whc/target/*.war /var/lib/jetty/webapps/whc.war
COPY --from=build /build/app/webapp-whc/jetty/config.xml /var/lib/jetty/webapps/config.xml

