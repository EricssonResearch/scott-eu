package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import eu.scott.warehouse.lib.LoggingMqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig;


public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);
    private static MqttClient mqttClient;

    /**
     * Begin listening on AdaptorHelper.lMQTT_TOPIC and perform a handshake with any twin that
     * registers there.
     */
    public static MqttClient initMqttClient() {
        try {
            final String mqttBroker = AdaptorHelper.p(WhcConfig.MQTT_TOPIC_PROP);
            mqttClient = new MqttClient(mqttBroker, AdaptorHelper.getMqttClientId());

            final MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
            mqttConnectOptions.setAutomaticReconnect(true);
            mqttClient.setCallback(new LoggingMqttCallback());
            // TODO Andrew@2018-03-13: set highest QoS
            mqttClient.connect(mqttConnectOptions);
            return mqttClient;
        } catch (MqttException e) {
            log.error("MQTT connection failed", e);

            // TODO Andrew@2018-07-30: exception strategy
            throw new IllegalStateException(e);
        }
    }

    public static void disconnect() {
        if(mqttClient != null) {
            try {
                mqttClient.disconnect();
            } catch (MqttException e) {
                log.warn("Unable to cleanly terminate the MQTT connection: ", e);
            }
        }
    }
}
