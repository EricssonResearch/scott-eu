import paho.mqtt.client as mqtt
import os,binascii
import logging
import time

CLIENT_ID = "robot-emulator-" + binascii.hexlify(os.urandom(15)).decode('utf-8')[:4]
BROKER_HOST = "mosquitto"

def main():
    logging.info("Client '%s' is connecting...", CLIENT_ID)
    # Client(client_id=””, clean_session=True, userdata=None, protocol=MQTTv311, transport=”tcp”)
    client = mqtt.Client(CLIENT_ID)
    try:
        client.connect(BROKER_HOST)
        logging.info("Client '%s' CONNECTED to '%s'", CLIENT_ID, BROKER_HOST)
    except Exception as e:
        logging.error("Failed to connect to the MQTT broker on host '%s' (CLIENT_ID='%s')", BROKER_HOST, CLIENT_ID)
        logging.debug(e)
    time.sleep(10)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%y-%m-%d %H:%M')
    main()
    logging.warning("Client '%s' is shutting down", CLIENT_ID)
