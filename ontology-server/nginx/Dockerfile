FROM ubuntu:16.04
LABEL maintainer "leonid.mokrushin@ericsson.com"

ARG PUBLIC_HOST
ARG PUBLIC_PORT

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install nginx
RUN apt-get -yq install nginx

# configure nginx
WORKDIR /etc/nginx/sites-available
COPY cliopatria .
RUN sed -i 's/%PUBLIC_HOST%/'"$PUBLIC_HOST"'/g' cliopatria
RUN sed -i 's/%PUBLIC_PORT%/'$PUBLIC_PORT'/g' cliopatria
# RUN cat cliopatria
WORKDIR /etc/nginx/sites-enabled
RUN ln -s /etc/nginx/sites-available/cliopatria .
RUN echo "daemon off;" >> /etc/nginx/nginx.conf

# expose port
EXPOSE $PUBLIC_PORT

# start ontology server
CMD ["service","nginx","start"]
