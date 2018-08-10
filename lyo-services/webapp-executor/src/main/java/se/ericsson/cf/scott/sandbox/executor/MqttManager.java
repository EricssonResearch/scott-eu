package se.ericsson.cf.scott.sandbox.executor;

import eu.scott.warehouse.MqttTopics;
import eu.scott.warehouse.domains.trs.LoggingMqttCallback;
import java.net.URI;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);
    private static MqttClient mqttClient;

    public static MqttClient getMqttClient() {
        return mqttClient;
    }

    public static MqttClient initMqttClient(final String mqttBroker, final String clientId) {
        try {
            mqttClient = new MqttClient(mqttBroker, clientId);
            final MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
            mqttConnectOptions.setAutomaticReconnect(true);
            mqttClient.setCallback(new LoggingMqttCallback());
            // TODO Andrew@2018-03-13: set highest QoS
            mqttClient.connect(mqttConnectOptions);
            mqttClient.subscribeWithResponse(
                    MqttTopics.WHC_PLANS,
                    new PlanPublicationListener(mqttClient)
            );
            return mqttClient;
        } catch (MqttException e) {
            log.error("MQTT connection failed", e);

            // TODO Andrew@2018-07-30: exception strategy
            throw new IllegalStateException(e);
        }
    }

}
