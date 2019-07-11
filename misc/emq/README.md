
Purpose: MQTT broker

To build: docker build -t emq .

To run: docker run -p:18083:18083 -tid emq

To access the web console:

http://localhost:18083/

Default username and password are: admin and public

Alternatively: http://emqtt.io/docs/v2/install.html#install-via-docker-image


How to test?

Full list of MQTT clients: https://github.com/mqtt/mqtt.github.io/wiki/libraries
