FROM scott/sandbox-base:latest as build

FROM jetty:9.4-jre11

COPY --from=build /build/app/webapp-whc/target/*.war /var/lib/jetty/webapps/ROOT.war
COPY --from=build /build/app/webapp-whc/jetty/config.xml /var/lib/jetty/webapps/
COPY --from=build /build/app/webapp-whc/jetty/override-web.xml /
