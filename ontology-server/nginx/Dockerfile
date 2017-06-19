FROM nginx
LABEL maintainer "leonid.mokrushin@ericsson.com"

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
