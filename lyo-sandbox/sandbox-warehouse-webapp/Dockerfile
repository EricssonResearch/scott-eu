FROM jetty:alpine

COPY target/sandbox-warehouse-webapp-*.war /var/lib/jetty/webapps/sandbox-warehouse.war
COPY sandbox-warehouse.xml /var/lib/jetty/webapps/sandbox-warehouse.xml
COPY override-web.xml /override-web.xml
