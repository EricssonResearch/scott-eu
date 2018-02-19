FROM jetty:alpine

COPY target/webapp-whc-*.war /var/lib/jetty/webapps/webapp.war
COPY jetty/config.xml /var/lib/jetty/webapps/
COPY jetty/override-web.xml /
