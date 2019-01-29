package se.ericsson.cf.scott.sandbox.twins.shelf.manager;

import com.google.common.collect.Lists;
import eu.scott.warehouse.lib.PlanChangeEventListener;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsConsumerConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.config.TrsProviderConfiguration;
import org.eclipse.lyo.oslc4j.trs.client.handlers.TrsProviderHandler;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsBasicAuthOslcClient;
import org.eclipse.lyo.oslc4j.trs.client.util.TrsConsumerUtils;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twins.shelf.AdaptorHelper;
import se.ericsson.cf.scott.sandbox.twins.shelf.ShelfTwinManager;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class TrsConsumerManager {
    private final static Logger log = LoggerFactory.getLogger(TrsConsumerManager.class);
    private static final String MQTT_CLIENT_ID = "trs-consumer-whc";
    private static final String warehouseTrsUri = "http://sandbox-whc:8080/services/trs";
    private static final Integer updateInterval = Integer.valueOf(
            AdaptorHelper.p("update.interval"));

    public static void initTrsConsumer() {
        final TrsConsumerConfiguration consumerConfig = buildConsumerConfig();
        final Collection<TrsProviderConfiguration> providerConfigs = buildProviderConfigs();

        final List<TrsProviderHandler> handlers = buildProviderHandlers(
                consumerConfig, providerConfigs);

        scheduleTrsHandlers(consumerConfig, handlers, getUpdateInterval());
    }

    public static Integer getUpdateInterval() {
        return updateInterval;
    }

    public static String getWarehouseTrsUri() {
        return warehouseTrsUri;
    }

    public static String getMqttClientId() {
        return MQTT_CLIENT_ID;
    }

    @NotNull
    private static TrsConsumerConfiguration buildConsumerConfig() {
        return new TrsConsumerConfiguration(AdaptorHelper.p("store.query"),
                                            // disable the SPARQL triplestore update
                                            null, null, null, new TrsBasicAuthOslcClient(),
                                            getMqttClientId(),
                                            // nothing fancy is really needed on the twins
                                            Executors.newSingleThreadScheduledExecutor()
        );
    }

    @NotNull
    private static List<TrsProviderHandler> buildProviderHandlers(
            final TrsConsumerConfiguration consumerConfig,
            final Collection<TrsProviderConfiguration> providerConfigs) {
        final List<TrsProviderHandler> handlers = TrsConsumerUtils.buildHandlersSequential(
                consumerConfig, providerConfigs);

        // attach our own listener to use "TRS everywhere"
        final PlanChangeEventListener listener = new PlanChangeEventListener(
                ShelfTwinManager.planExecutorSvc);
        for (TrsProviderHandler handler : handlers) {
            handler.attachListener(listener);
        }
        return handlers;
    }

    @NotNull
    private static Collection<TrsProviderConfiguration> buildProviderConfigs() {
        // FIXME Andrew@2018-06-19: extract to a property
        final String warehouseTrsUri = TrsConsumerManager.getWarehouseTrsUri();
        final String basicAuthUsername = null;
        final String basicAuthPassword = null;
        final String mqttBroker = AdaptorHelper.p("trs.mqtt.broker");
        final String mqttTopic = AdaptorHelper.p("trs.mqtt.topic");
        return Lists.newArrayList(
                new TrsProviderConfiguration(warehouseTrsUri, basicAuthUsername, basicAuthPassword,
                                             mqttBroker, mqttTopic, null
                ));
    }

    private static void scheduleTrsHandlers(final TrsConsumerConfiguration consumerConfig,
            final List<TrsProviderHandler> handlers, final Integer updateInterval) {
        final Random random = new Random(System.currentTimeMillis());
        for (TrsProviderHandler handler : handlers) {
            consumerConfig.getScheduler()
                          .scheduleAtFixedRate(handler, random.nextInt(updateInterval),
                                               updateInterval, TimeUnit.SECONDS
                          );
        }
    }

}
