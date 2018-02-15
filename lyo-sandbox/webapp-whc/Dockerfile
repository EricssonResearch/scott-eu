FROM jetty:alpine

COPY target/webapp-whc-*.war /var/lib/jetty/webapps/sandbox-whc.war
COPY jetty/sandbox-whc.xml /var/lib/jetty/webapps/sandbox-whc.xml
COPY jetty/override-web.xml /override-web.xml
