FROM scott/sandbox-base:latest as build

FROM jetty:9.4-alpine

# WARNING DO NOT CHANGE WORKDIR or set it back to what it was before
# $JETTY_BASE must be correct before starting Jetty

COPY --from=build /build/app/webapp-svc-location/target/*.war /var/lib/jetty/webapps/ROOT.war
