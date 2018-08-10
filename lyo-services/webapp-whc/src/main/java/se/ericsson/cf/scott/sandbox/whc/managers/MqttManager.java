package se.ericsson.cf.scott.sandbox.whc.managers;

import com.google.common.collect.ImmutableMap;
import eu.scott.warehouse.MqttTopics;
import eu.scott.warehouse.domains.trs.LoggingMqttCallback;
import java.net.URI;
import java.util.Map;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.planning.PlanDTO;
import se.ericsson.cf.scott.sandbox.whc.trs.TwinRegistrationListener;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);
    private static MqttClient mqttClient;
    private static TwinRegistrationListener registrationListener;

    public static Map<String, URI> getTwins() {
        return ImmutableMap.copyOf(registrationListener.getTwins());
    }

    public static ImmutableMap<String, URI> getExecutors() {
        return ImmutableMap.copyOf(registrationListener.getExecutors());
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
            registrationListener = new TwinRegistrationListener(mqttClient, MqttTopics.WHC_PLANS);
            mqttClient.subscribeWithResponse(MqttTopics.REGISTRATION_ANNOUNCE, registrationListener
            );
            return mqttClient;
        } catch (MqttException e) {
            log.error("MQTT connection failed", e);

            // TODO Andrew@2018-07-30: exception strategy
            throw new IllegalStateException(e);
        }
    }

    public static void triggerPlan(final PlanDTO planDTO) {
//        URI exec = pickExecutor()
        throw new UnsupportedOperationException("Not implemented yet");
    }
}
