import paho.mqtt.client as mqtt
import os,binascii
import logging
import time

ID_STRING = binascii.hexlify(os.urandom(15)).decode('utf-8')[:4]
CLIENT_ID = "robot-emulator-" + ID_STRING
BROKER_HOST = "mosquitto"
TOPIC_STATUS = "twin/%s/status" % ID_STRING
TOPIC_PLANS = "twin/%s/plans" % ID_STRING
TOPIC_REGISTRATION = "twins/registration"

def main():
    logging.info("Client '%s' is connecting...", CLIENT_ID)
    # Client(client_id=””, clean_session=True, userdata=None, protocol=MQTTv311, transport=”tcp”)
    client = mqtt.Client(CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(BROKER_HOST)
        logging.info("Client '%s' CONNECTED to '%s'", CLIENT_ID, BROKER_HOST)
    except Exception as e:
        logging.error("Failed to connect to the MQTT broker on host '%s' (CLIENT_ID='%s')", BROKER_HOST, CLIENT_ID)
        logging.debug(e)
    client.loop_forever()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.


    client.subscribe(TOPIC_STATUS)
    client.subscribe(TOPIC_PLANS)
    logging.info("Subscribed to %s", TOPIC_STATUS)
    logging.info("Subscribed to %s", TOPIC_PLANS)

    client.publish(TOPIC_STATUS, "{'status': 'on'}")
    client.publish(TOPIC_REGISTRATION, "{'twin': '%s'}" % ID_STRING)

    # TODO also publish some message on the 'registration' topic

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    logging.debug("New message '%' (topic: '%s', QoS%d)", msg.payload, msg.topic, msg.qos)
    if not msg.topic == TOPIC_STATUS:
        client.publish(TOPIC_STATUS, "{'status': 'done'}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%y-%m-%d %H:%M')
    main()
    logging.warning("Client '%s' is shutting down", CLIENT_ID)
