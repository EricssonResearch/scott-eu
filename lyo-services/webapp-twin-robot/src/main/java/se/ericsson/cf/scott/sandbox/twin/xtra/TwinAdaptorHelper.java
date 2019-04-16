package se.ericsson.cf.scott.sandbox.twin.xtra;

import com.google.common.base.Strings;
import eu.scott.warehouse.lib.MqttClientBuilder;
import eu.scott.warehouse.lib.MqttTopics;
import eu.scott.warehouse.lib.TrsMqttGateway;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.UUID;
import javax.servlet.ServletContext;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.oslc4j.client.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
import org.eclipse.lyo.oslc4j.trs.server.ChangeHistories;
import org.eclipse.lyo.store.Store;
import org.eclipse.lyo.store.StoreFactory;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import se.ericsson.cf.scott.sandbox.twin.TwinManager;
import se.ericsson.cf.scott.sandbox.twin.TwinsServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.twin.servlet.ServiceProviderCatalogSingleton;
import se.ericsson.cf.scott.sandbox.twin.servlet.TwinsServiceProvidersFactory;
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.TrsMqttClientManager;
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.TwinAckRegistrationAgent;
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.TwinChangeHistories;


public class TwinAdaptorHelper {
    private final static Logger log = LoggerFactory.getLogger(TwinAdaptorHelper.class);
    private final static UUID uuid = UUID.randomUUID();
    static String trsTopic;
    static TrsMqttClientManager trsClientManager;
    static Store store;
    static TrsMqttGateway mqttGateway;
    static TwinChangeHistories changeHistories;
    private static ServletContext servletContext;
    private static ServiceProviderRepository serviceProviderRepository;

    public static TrsMqttGateway getMqttGateway() {
        return mqttGateway;
    }

    public static void setMqttGateway(final TrsMqttGateway mqttGateway) {
        TwinAdaptorHelper.mqttGateway = mqttGateway;
    }

    @NotNull
    public static String getTwinUUID() {
        return "twn-" + uuid.toString();
    }

    public static String getTrsTopic() {
        if (Strings.isNullOrEmpty(trsTopic)) {
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

    public static void setChangeHistories(final TwinChangeHistories changeHistories) {
        TwinAdaptorHelper.changeHistories = changeHistories;
    }

    public static ServletContext getServletContext() {
        return servletContext;
    }

    public static void setServletContext(final ServletContext servletContext) {
        TwinAdaptorHelper.servletContext = servletContext;
    }

    public static ServiceProviderRepository getTwins() {
        // FIXME Andrew@2019-01-21: persist a single instance
        return new ServiceProviderRepositoryStoreImpl(getStore(), getTwinsGraphURI());
    }

    // TODO Andrew@2019-01-23: move to some other singleton, ideally use DI
    public static ServiceProviderRepository getServiceProviderRepository() {
        if (serviceProviderRepository == null) {
            serviceProviderRepository = new ServiceProviderRepositoryStoreImpl(getStore(), getTwinsGraphURI());
        }
        return serviceProviderRepository;
    }

    public static String p(final String s) {
        return getServletContext().getInitParameter(parameterFQDN(s));
    }

    @NotNull
    public static TwinRegistrationClient createTwinRegistrationClient() {
        final OslcClient client = new OslcClient();
        // TODO Andrew@2019-01-29: remove hardcoded URI
        return new TwinRegistrationClient(
            client, "http://sandbox-whc:8080/services/service2/registrationMessages/register");
    }

    public static void initTrsClient() {
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
        final String query_endpoint = p("store.query");
        final String update_endpoint = p("store.update");
        initStoreGeneric(query_endpoint, update_endpoint, wipeOnStartup);
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

    private static void setStore(final Store store) {
        TwinAdaptorHelper.store = store;
    }

    // Start of user code class_methods
    private static String parameterFQDN(final String s) {
        return TwinManager.PACKAGE_ROOT + "." + s;
    }

    static ServiceProvider registerProvider(final TwinsServiceProviderInfo info) {
        try {
            log.info("Registering provider: {}", info);
            final ServiceProvider robotSP = TwinsServiceProvidersFactory.createTwinsServiceProvider(info);
            ServiceProviderCatalogSingleton.registerTwinsServiceProvider(robotSP);
            return robotSP;
        } catch (URISyntaxException | OslcCoreApplicationException e) {
            log.error("Cannot register the Robot SP", e);
            // TODO Andrew@2018-09-04: strategy w/Leo
            return null;
        }
    }

    private static void initStoreGeneric(final String query_endpoint, final String update_endpoint,
        final boolean wipeOnStartup) {
        log.info("Initialising Lyo Store");
        log.debug("SPARQL endpoints: query={}; update={}", query_endpoint, update_endpoint);
        try {
            final Store store = StoreFactory.sparql(query_endpoint, update_endpoint);
            if (wipeOnStartup) {
                log.warn("Erasing the whole dataset");
                store.removeAll();
            }
            setStore(store);
        } catch (IOException | ARQException e) {
            log.error("Failed to initialise Lyo Store with SPARQL endpoints: query={}; update={}", query_endpoint,
                      update_endpoint
            );
            // TODO Andrew@2018-07-29: rethink exception management
            throw new IllegalStateException(e);
        }
    }
}
