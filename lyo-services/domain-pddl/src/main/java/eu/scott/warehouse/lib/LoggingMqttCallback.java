package eu.scott.warehouse.lib;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class LoggingMqttCallback implements MqttCallbackExtended {
    private final static Logger log = LoggerFactory.getLogger(LoggingMqttCallback.class);

    @Override
    public void connectionLost(final Throwable cause) {
        log.warn("Connection lost");
        log.trace("Connection lost", cause);
    }

    @Override
    public void messageArrived(final String topic, final MqttMessage message) throws Exception {
        log.trace("Message {} arrived from {}", message.getId(), topic);
    }

    @Override
    public void deliveryComplete(final IMqttDeliveryToken token) {
        if (token == null) {
            log.debug("Delivery complete (message ID could not be retrieved from the token)");
        } else {
            try {
                final MqttMessage message = token.getMessage();
                if (message != null) {
                    log.trace("Delivery complete for message {}", message.getId());
                } else {
                    log.debug(
                        "Delivery complete (message ID could not be retrieved from the token)");
                }
            } catch (MqttException e) {
                log.warn("Delivery complete (message ID could not be retrieved from the token)");
            }

        }
    }

    @Override
    public void connectComplete(final boolean reconnect, final String serverURI) {
        log.info("{} to the MQTT broker", reconnect ? "Reconnected" : "Connected");
    }


}
