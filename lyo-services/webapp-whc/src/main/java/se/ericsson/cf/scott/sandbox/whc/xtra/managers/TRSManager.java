package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import com.google.common.collect.Lists;
import java.time.Duration;
import java.util.Collection;
import java.util.concurrent.Executors;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsConsumerConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsProviderConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsBasicAuthOslcClient;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsConsumerUtils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.trs.WhcChangeHistories;

/**
 * TODO
 *
 * @version $version-stub$
 * @since TODO
 */
public class TRSManager {

    private final static Logger log = LoggerFactory.getLogger(TRSManager.class);

    public static void initTRSClient(final MqttClient mqttClient) {
        final TrsConsumerConfiguration consumerConfig = new TrsConsumerConfiguration(
            AdaptorHelper.p(AdaptorHelper.KB_QUERY_PROP),
            AdaptorHelper.p(AdaptorHelper.KB_UPDATE_PROP), null, null, new TrsBasicAuthOslcClient(),
            AdaptorHelper.getMqttClientIdStatic(), Executors.newSingleThreadScheduledExecutor());
        final Collection<TrsProviderConfiguration> providerConfigs = Lists.newArrayList(
            TrsProviderConfiguration.forMQTT(mqttClient,
                AdaptorHelper.p(AdaptorHelper.MQTT_TOPIC_PROP)));
        TrsConsumerUtils.buildHandlersSequential(consumerConfig, providerConfigs);
    }

    public static void initTRSServer(final MqttClient mqttClient) {
        // TODO Andrew@2018-07-18: figure out how the change history works over MQTT
        WarehouseControllerManager.setChangeHistories(
            new WhcChangeHistories(mqttClient, AdaptorHelper.p(AdaptorHelper.MQTT_TOPIC_PROP),
                Duration.ofMinutes(5).toMillis()));
    }

}
