package se.ericsson.cf.scott.sandbox.whc.xtra.managers;

import com.google.common.collect.Lists;
import java.util.Collection;
import java.util.concurrent.Executors;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsConsumerConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsProviderConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsConsumerUtils;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.whc.xtra.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.whc.xtra.WhcConfig;


public class TRSManager {

    private final static Logger log = LoggerFactory.getLogger(TRSManager.class);

    public static void initTRSClient(final MqttClient mqttClient) {
        final TrsConsumerConfiguration consumerConfig = new TrsConsumerConfiguration(
            AdaptorHelper.p(WhcConfig.KB_QUERY_PROP), AdaptorHelper.p(WhcConfig.KB_UPDATE_PROP),
            null, null, AdaptorHelper.getMqttClientIdStatic(),
            Executors.newSingleThreadScheduledExecutor(), null, null);
        final Collection<TrsProviderConfiguration> providerConfigs = Lists.newArrayList(
            TrsProviderConfiguration.forMQTT(mqttClient,
                AdaptorHelper.p(WhcConfig.MQTT_TOPIC_PROP)));
        TrsConsumerUtils.buildHandlersSequential(consumerConfig, providerConfigs);
    }
}
