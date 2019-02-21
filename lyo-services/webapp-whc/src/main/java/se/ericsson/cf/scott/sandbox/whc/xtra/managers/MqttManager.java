package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import com.google.common.collect.ImmutableMap;
import eu.scott.warehouse.lib.MqttTopics;
import eu.scott.warehouse.lib.LoggingMqttCallback;
import java.net.URI;
import java.util.Map;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.xtra.trs.TwinRegistrationListener;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);
    private static MqttClient mqttClient;
    private static TwinRegistrationListener registrationListener;

    // TODO Andrew@2019-02-21: move to the repository
    public static Map<String, URI> getTwins() {
        return ImmutableMap.copyOf(registrationListener.getTwins());
    }

    // FIXME Andrew@2019-02-21: delete in favour of se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager.getExecService
    public static ImmutableMap<String, URI> getExecutors() {
        return ImmutableMap.copyOf(registrationListener.getExecutors());
    }

    public static MqttClient getMqttClient() {
        return mqttClient;
    }

    /**
     * Begin listening on AdaptorHelper.MQTT_TOPIC and perform a handshake with any twin that
     * registers there.
     */
    public static MqttClient initMqttClient() {
        try {
            final String mqttBroker = AdaptorHelper.p(AdaptorHelper.MQTT_TOPIC_PROP);
            mqttClient = new MqttClient(mqttBroker, WarehouseControllerManager.getMqttClientId());

            final MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
            mqttConnectOptions.setAutomaticReconnect(true);
            mqttClient.setCallback(new LoggingMqttCallback());
            // TODO Andrew@2018-03-13: set highest QoS
            mqttClient.connect(mqttConnectOptions);

            registrationListener = new TwinRegistrationListener(mqttClient, MqttTopics.WHC_PLANS);
            mqttClient.subscribe(MqttTopics.REGISTRATION_ANNOUNCE, registrationListener);

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
