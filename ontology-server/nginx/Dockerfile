FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install nginx
RUN apt-get -yq install nginx

# configure nginx
WORKDIR /etc/nginx/sites-available
COPY cliopatria .
WORKDIR /etc/nginx/sites-enabled
RUN ln -s /etc/nginx/sites-available/cliopatria .
RUN echo "daemon off;" >> /etc/nginx/nginx.conf

# expose port
EXPOSE 80

# start ontology server
CMD service nginx start
