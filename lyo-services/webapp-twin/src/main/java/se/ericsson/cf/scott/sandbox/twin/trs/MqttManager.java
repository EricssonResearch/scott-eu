package se.ericsson.cf.scott.sandbox.twin.trs;

import eu.scott.warehouse.MqttHelper;
import eu.scott.warehouse.MqttTopics;
import eu.scott.warehouse.domains.trs.LoggingMqttCallback;
import eu.scott.warehouse.domains.trs.TrsServerAnnouncement;
import eu.scott.warehouse.domains.trs.TrsXConstants;
import java.net.URI;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import javax.ws.rs.core.UriBuilder;
import org.eclipse.lyo.oslc4j.core.OSLC4JUtils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.twin.RobotTwinManager;

/**
 * TBD
 *
 * @version $version-stub$
 * @since FIXME
 */
public class MqttManager {
    private final static Logger log = LoggerFactory.getLogger(MqttManager.class);

}
