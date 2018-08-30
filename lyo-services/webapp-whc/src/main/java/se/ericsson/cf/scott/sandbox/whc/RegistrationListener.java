package se.ericsson.cf.scott.sandbox.whc;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.ObjectWriter;
import com.fasterxml.jackson.databind.node.ObjectNode;
import eu.scott.warehouse.lib.MqttTopics;
import java.io.ByteArrayInputStream;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created on 2018-07-18
 *
 * @author Andrew Berezovskyi (andriib@kth.se)
 * @version $version-stub$
 * @since 0.0.1
 */
public class RegistrationListener implements IMqttMessageListener {
    private static final String STATUS_AWAITING = "awaiting";
    private static final Logger log = LoggerFactory.getLogger(RegistrationListener.class);
    private final ObjectMapper om;
    private final MqttClient mqttClient;

    public RegistrationListener(final MqttClient mqttClient) {
        this.mqttClient = mqttClient;
        om = new ObjectMapper();
    }

    @Override
    public void messageArrived(final String topic, final MqttMessage message) throws Exception {
        try {
            log.trace("New registration message in topic '{}'", topic);
            final ObjectReader reader = om.reader();
            final JsonNode receivedMessage = reader.readTree(
                    new ByteArrayInputStream(message.getPayload()));
            log.trace(receivedMessage.toString());
            final String status = receivedMessage.get("status").asText();
            log.debug("New message in the registration topic with the status {}", status);
            if (STATUS_AWAITING.equals(status)) {
                final String twinId = receivedMessage.get("twin").asText();
                log.info("Responding to the registration request from the device");
                final ObjectNode objectNode = om.createObjectNode();
                objectNode.put("device", twinId);
                objectNode.put("whc", WarehouseControllerManager.getWhcId());
                final ObjectWriter writer = om.writer();
                final byte[] payloadBytes = writer.writeValueAsBytes(objectNode);
                final MqttMessage responseMessage = new MqttMessage(payloadBytes);
                mqttClient.publish(MqttTopics.TWIN_SIM_REG_ACK, responseMessage);
            }
        } catch (Exception e) {
            log.error("Exception processing the registration message: ", e);
        }
    }
}
