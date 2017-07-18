FROM busybox:latest

VOLUME /fuseki/configuration

ADD ./configuration /tempfolder
RUN mkdir -p /fuseki/configuration

CMD cp /tempfolder/* /fuseki/configuration