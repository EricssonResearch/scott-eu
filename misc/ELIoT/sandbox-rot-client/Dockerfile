FROM mhart/alpine-node

RUN mkdir -p /app/src/
#&& cp -a /tmp/node_modules /app/
WORKDIR /app


ADD package.json /app
ADD package-lock.json /app

ADD server.js /app

# ADD _config.yml /app
# ADD .env /app
ADD data.json /app


RUN npm install

# Application's default ports
EXPOSE 5683

ENTRYPOINT [ "npm", "start", "sandbox-leshan-server", "-t" ]
