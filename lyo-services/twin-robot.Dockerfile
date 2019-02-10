FROM scott/sandbox-base:latest as build

# TODO 9.4 (might have to wait till JDK9+ & Jersey w/JAX-RS 2.1)
# https://github.com/eclipse/jetty.project/issues/2412
# https://github.com/eclipse/lyo.core/issues/31
FROM jetty:9.4-jre11

# WARNING DO NOT CHANGE WORKDIR or set it back to what it was before
# $JETTY_BASE must be correct before starting Jetty

COPY --from=build /build/app/webapp-twin-robot/target/*.war /var/lib/jetty/webapps/ROOT.war
