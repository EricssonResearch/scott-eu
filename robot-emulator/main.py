import paho.mqtt.client as mqtt
import os,binascii
import logging
import time
from enum import Enum
from threading import Timer
import json
import random
import math

ID_STRING = binascii.hexlify(os.urandom(15)).decode('utf-8')[:4]
CLIENT_ID = "robot-emulator-" + ID_STRING
BROKER_HOST = "mosquitto"
TOPIC_STATUS = "twin/%s/status" % ID_STRING
TOPIC_PLANS = "twin/%s/plans" % ID_STRING
TOPIC_REGISTRATION = "twins/registration/announce"
TOPIC_HANDSHAKE = "twins/registration/handshake"

class TwinStatus(Enum):
    NOT_CONNECTED = 1
    SEARCHING = 2
    SELECTED = 3
    CONNECTED = 4
    DISCONNECTED = 5

status = TwinStatus.NOT_CONNECTED
timer = None

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

def twin_search_timeout(client, n):
    if not status == TwinStatus.CONNECTED:
        logging.warning("Twin connection is not established (%s)", status)
        request_twin(client)
        schedule_reconnect(client, n+1)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.

    # no need to sub to our own statuses
    # sub(client, TOPIC_STATUS)
    sub(client, TOPIC_PLANS)
    sub(client, TOPIC_HANDSHAKE)

    # client.publish(TOPIC_STATUS, "{'status': 'on'}")
    request_twin(client)
    schedule_reconnect(client, 1)

    # TODO also publish some message on the 'registration' topic

def sub(client, topic):
    client.subscribe(topic)
    logging.info("Subscribed to %s", topic)


def schedule_reconnect(client, n):
    delay = min(0.1 * 2 ** (n-1) + (random.randint(0, 200) / 1000), 10)
    logging.debug("Next reconnection attempt in %fs", delay)
    timer = Timer(delay, twin_search_timeout, [client, n])
    timer.start()

def request_twin(client):
    client.publish(TOPIC_REGISTRATION, json.dumps({'twin': ID_STRING, 'status': 'awaiting'}))
    status = TwinStatus.SEARCHING


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    logging.debug("New message '%s' (topic: '%s', QoS%d)", msg.payload, msg.topic, msg.qos)
    if not msg.topic == TOPIC_STATUS:
        client.publish(TOPIC_STATUS, json.dumps({'status': 'done'}))
    if msg.topic == TOPIC_HANDSHAKE:
        reg_reply = json.loads(msg.payload)
        process_reg_reply(reg_reply, client, msg)

def process_reg_reply(reg_reply, client, msg):
    if reg_reply["device"] != ID_STRING:
        logging.debug("A registration message for another device received: %s", msg.payload)
    else:
        t = reg_reply["twin"]
        logging.debug("Trying to select the twin '%s'", t)
        # TODO do we really need this status?
        status = TwinStatus.SELECTED
        register_with_twin(t)

def register_with_twin(t):
    logging.warning("Not implemented yet")
    status = TwinStatus.CONNECTED
    twin = t

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%y-%m-%d %H:%M')
    main()
    logging.warning("Client '%s' is shutting down", CLIENT_ID)
