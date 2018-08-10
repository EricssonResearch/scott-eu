FROM berezovskyi/scott-sandbox-base:latest as build


# 9.4 might have to wait till JDK9
# https://github.com/eclipse/jetty.project/issues/2412
# FROM jetty:9.3-jre8
FROM jetty:9.3-jre8-alpine

# WARNING DO NOT CHANGE WORKDIR or set it back to what it was before
# $JETTY_BASE must be correct before starting Jetty

COPY --from=build /build/app/webapp-executor/target/*.war /var/lib/jetty/webapps/ROOT.war
