package se.ericsson.cf.scott.sandbox.twin;

import com.google.common.base.Strings;
import com.google.common.collect.ImmutableList;
import eu.scott.warehouse.lib.MqttClientBuilder;
import eu.scott.warehouse.lib.MqttTopics;
import eu.scott.warehouse.lib.TrsMqttGateway;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.UUID;
import javax.servlet.ServletContext;
import org.eclipse.lyo.client.oslc.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.store.Store;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.clients.TwinRegistrationClient;
// TODO Andrew@2019-01-22: clean up
//import se.ericsson.cf.scott.sandbox.twin.ros.RosManager;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twin.servlet.TwinsServiceProvidersFactory;
import se.ericsson.cf.scott.sandbox.twin.trs.LyoStoreManager;
import se.ericsson.cf.scott.sandbox.twin.trs.TrsMqttClientManager;
import se.ericsson.cf.scott.sandbox.twin.trs.TwinAckRegistrationAgent;
import se.ericsson.cf.scott.sandbox.twin.trs.TwinChangeHistories;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class TwinAdaptorHelper {
    private final static Logger log = LoggerFactory.getLogger(TwinAdaptorHelper.class);
    private final static UUID uuid = UUID.randomUUID();
    static String trsTopic;
    static TrsMqttClientManager trsClientManager;
    static Store store;
    static TrsMqttGateway mqttGateway;
//    private static HazelcastInstance hc;
//    static IMap<String, TwinsServiceProviderInfo> twinProviderInfo;
    static TwinChangeHistories changeHistories;
    static ServletContext servletContext;
    private static ServiceProviderRepository serviceProviderRepository;

    // Start of user code class_methods
    @NotNull
    public static String getTwinUUID() {
        return "twn-" + uuid.toString();
    }

    public static String getTrsTopic() {
        if(Strings.isNullOrEmpty(trsTopic)) {
            log.warn("The TRS topic was requested before it was set");
        }
        return trsTopic;
    }

    public static void setTrsTopic(String trsTopic) {
        TwinAdaptorHelper.trsTopic = trsTopic;
    }

    public static TrsMqttClientManager getTrsClientManager() {
        return trsClientManager;
    }

    public static void setTrsClientManager(TrsMqttClientManager trsClientManager) {
        TwinAdaptorHelper.trsClientManager = trsClientManager;
    }

    public static ChangeHistories getChangeHistories() {
        return changeHistories;
    }

    public static ServletContext getServletContext() {
        return servletContext;
    }

    public static ServiceProviderRepository getTwins() {
        // FIXME Andrew@2019-01-21: persist a single instance
        return new ServiceProviderRepositoryStoreImpl(getStore(), getTwinsGraphURI());
    }

    private static URI getTwinsGraphURI() {
        return URI.create("http://scott.example.com/g/twinSPs");
    }

    private static Store getStore() {
//        return StoreFactory.inMemory();
        if (store == null) {
            log.warn("Lyo Store was not initialised properly");
            initStore(false);
        }
        return store;
    }

    // TODO Andrew@2019-01-23: move to some other singleton, ideally use DI
    public static ServiceProviderRepository getServiceProviderRepository() {
        if(serviceProviderRepository == null) {
            serviceProviderRepository = new ServiceProviderRepositoryStoreImpl(getStore(), getTwinsGraphURI());
        }
        return serviceProviderRepository;
    }

    private static void setStore(final Store store) {
        TwinAdaptorHelper.store = store;
    }

    // Start of user code class_methods
    private static String parameterFQDN(final String s) {
        return TwinManager.PACKAGE_ROOT + "." + s;
    }

    public static String p(final String s) {
        return getServletContext().getInitParameter(parameterFQDN(s));
    }

    @NotNull
    public static TwinRegistrationClient createTwinRegistrationClient() {
        // FIXME Andrew@2018-09-04: rewrite & call from the CF handler
        final OslcClient client = new OslcClient();
        // TODO Andrew@2019-01-29: remove hardcoded URI
        return new TwinRegistrationClient(
            client, "http://sandbox-whc:8080/services/service2/registrationMessages/register");
    }

    static ServiceProvider registerProvider(final TwinsServiceProviderInfo info) {
        try {
            log.info("Registering provider: {}", info);
            final ServiceProvider robotSP = TwinsServiceProvidersFactory.createTwinsServiceProvider(
                info);
            ServiceProviderCatalogSingleton.registerTwinsServiceProvider(robotSP);
            return robotSP;
        } catch (URISyntaxException | OslcCoreApplicationException e) {
            log.error("Cannot register the Robot SP", e);
            // TODO Andrew@2018-09-04: strategy w/Leo
            return null;
        }
    }

    static void initTrsClient() {
        final String mqttBroker = p("trs.mqtt.broker");
        // TODO Andrew@2018-07-31: remove non-gateway based code
        try {
            log.debug("Connecting to the MQTT broker: {}", mqttBroker);
            mqttGateway = new MqttClientBuilder().withBroker(mqttBroker)
                                                 .withId(getTwinUUID())
                                                 .withRegistration(new TwinAckRegistrationAgent(MqttTopics.WHC_PLANS))
                                                 .build();
            final MqttClient mqttClient = mqttGateway.getMqttClient();
            final TrsMqttClientManager trsClientManager = new TrsMqttClientManager(mqttClient);
            setTrsClientManager(trsClientManager);
            new Thread(trsClientManager::connectAndSubscribeToPlans).run();
        } catch (MqttException e) {
            log.error("Failed to initialise the MQTT gateway", e);
        }
    }

    public static void initStore(boolean wipeOnStartup) {
        final Store store = LyoStoreManager.initLyoStore(wipeOnStartup);
        setStore(store);
    }
}
