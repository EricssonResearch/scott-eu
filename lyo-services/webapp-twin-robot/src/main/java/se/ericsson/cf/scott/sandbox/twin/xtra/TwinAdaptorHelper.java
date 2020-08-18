package se.ericsson.cf.scott.sandbox.twin.xtra;

import eu.scott.warehouse.lib.MqttGatewayBuilder;
import eu.scott.warehouse.lib.RdfMqttGateway;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.UUID;
import javax.servlet.ServletContext;
import org.apache.jena.sparql.ARQException;
import org.eclipse.lyo.oslc4j.client.OslcClient;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.ServiceProvider;
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
import se.ericsson.cf.scott.sandbox.twin.xtra.repository.ExecutionReportRepository;
import se.ericsson.cf.scott.sandbox.twin.xtra.trs.TrsMqttPlanManager;

public class TwinAdaptorHelper {
    private final static Logger log = LoggerFactory.getLogger(TwinAdaptorHelper.class);
    private final static UUID uuid = UUID.randomUUID();
    private static TrsMqttPlanManager trsClientManager;
    private static Store store;
    private static RdfMqttGateway mqttGateway;
    private static ServletContext servletContext;
    private static ServiceProviderRepository serviceProviderRepository;
    private static ExecutionReportRepository executionReportRepository;

    public static RdfMqttGateway getMqttGateway() {
        return mqttGateway;
    }

    @NotNull
    public static String getTwinUUID() {
        return "twn-" + uuid.toString();
    }

    public static TrsMqttPlanManager getTrsClientManager() {
        return trsClientManager;
    }

    public static void setTrsClientManager(TrsMqttPlanManager trsClientManager) {
        TwinAdaptorHelper.trsClientManager = trsClientManager;
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
            // TODO Andrew@2019-07-16: thread safety
            serviceProviderRepository = new ServiceProviderRepositoryStoreImpl(getStore(), getTwinsGraphURI());
        }
        return serviceProviderRepository;
    }

    public static ExecutionReportRepository getExecutionReportRepository() {
        if (executionReportRepository == null) {
            executionReportRepository = new ExecutionReportRepository(getStore());
        }
        return executionReportRepository;
    }


    public static String p(final String s) {
        return getServletContext().getInitParameter(parameterFQDN(s));
    }

    @NotNull
    public static TwinRegistrationClient createTwinRegistrationClient() {
        final OslcClient client = new OslcClient();
        // TODO Andrew@2019-01-29: remove hardcoded URI
        return new TwinRegistrationClient(
            client, "http://whc.svc:8080/services/service2/registrationMessages/register");
    }

    public static void initTrsClient() {
        // TODO Andrew@2018-07-31: remove non-gateway based code
        final MqttClient mqttClient = getMqttClient();
        final TrsMqttPlanManager trsClientManager = new TrsMqttPlanManager(mqttClient);
        setTrsClientManager(trsClientManager);
        new Thread(trsClientManager::connectAndSubscribeToPlans).run();
    }

    @NotNull
    public static MqttClient getMqttClient() {
        if (mqttGateway == null) {
            initMqttGateway();
        }
        return mqttGateway.getMqttClient();
    }

    private static void initMqttGateway() {
        try {
            final String mqttBroker = p("trs.mqtt.broker");
            log.debug("Connecting to the MQTT broker: {}", mqttBroker);
            mqttGateway = new MqttGatewayBuilder().withBroker(mqttBroker).withId(getTwinUUID()).build();
        } catch (MqttException e) {
            log.error("Cannot connect to the MQTT broker");
            throw new RuntimeException(e);
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

    public static Store getStore() {
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
            final ServiceProvider robotSP = TwinsServiceProvidersFactory.createServiceProvider(info);
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
        } catch (ARQException e) {
            log.error("Failed to initialise Lyo Store with SPARQL endpoints: query={}; update={}", query_endpoint,
                      update_endpoint
            );
            // TODO Andrew@2018-07-29: rethink exception management
            throw new IllegalStateException(e);
        }
    }
}
