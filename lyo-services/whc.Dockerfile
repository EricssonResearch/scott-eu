FROM berezovskyi/scott-sandbox-base:latest as build

# TODO 9.4 (might have to wait till JDK9+)
# https://github.com/eclipse/jetty.project/issues/2412
# TODO jetty:9.3-jre8-alpine (see twin.Dockerfile)
FROM jetty:9.4-jre8-alpine

COPY --from=build /build/app/webapp-whc/target/*.war /var/lib/jetty/webapps/ROOT.war
