package se.ericsson.cf.scott.sandbox.whc.managers;

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
import se.ericsson.cf.scott.sandbox.whc.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.trs.TwinRegistrationListener;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);
    private static Map<String, URI> twins = new ConcurrentHashMap<>();
    private static MqttClient mqttClient;

    public static Map<String, URI> getTwins() {
        return twins;
    }

    public static MqttClient getMqttClient() {
        return mqttClient;
    }

    public static MqttClient initMqttClient() {
        try {
            final String mqttBroker = WarehouseControllerManager.p(AdaptorHelper.MQTT_TOPIC);
            mqttClient = new MqttClient(mqttBroker, WarehouseControllerManager.getWhcId());
            final MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
            mqttConnectOptions.setAutomaticReconnect(true);
            mqttClient.setCallback(new LoggingMqttCallback());
            // TODO Andrew@2018-03-13: set highest QoS
            mqttClient.connect(mqttConnectOptions);
            mqttClient.subscribeWithResponse(
                    MqttTopics.REGISTRATION_ANNOUNCE,
                    new TwinRegistrationListener(twins, mqttClient, MqttTopics.WHC_PLANS)
            );
            return mqttClient;
        } catch (MqttException e) {
            log.error("MQTT connection failed", e);

            // TODO Andrew@2018-07-30: exception strategy
            throw new IllegalStateException(e);
        }
    }

}
