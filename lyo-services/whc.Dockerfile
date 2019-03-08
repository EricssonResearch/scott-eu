FROM scott/sandbox-base:latest as build

FROM jetty:9.4-alpine

COPY --from=build /build/app/webapp-whc/target/*.war /var/lib/jetty/webapps/whc.war
COPY --from=build /build/app/webapp-whc/jetty/whc.xml /var/lib/jetty/webapps/whc.xml
COPY --from=build /build/app/webapp-whc/jetty/override-web.xml /var/lib/jetty/override-web.xml
