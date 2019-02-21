package se.ericsson.cf.scott.sandbox.whc.xtra;

import java.io.InputStream;
import java.util.logging.Level;
import javax.servlet.ServletContext;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.rdf.model.ModelFactory;
import org.apache.jena.riot.Lang;
import org.apache.jena.riot.RDFDataMgr;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.slf4j.bridge.SLF4JBridgeHandler;
import se.ericsson.cf.scott.sandbox.whc.ServiceProviderInfo;
import se.ericsson.cf.scott.sandbox.whc.WarehouseControllerManager;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class AdaptorHelper {
    public static final String KB_QUERY_PROP = "kb.query_uri";
    private final static Logger log = LoggerFactory.getLogger(AdaptorHelper.class);
    // Start of user code class_attributes
    private final static String PACKAGE_ROOT = WarehouseControllerManager.class.getPackage().getName();
    // TODO Andrew@2018-07-30: extract into AdaptorConfig (non-static)
    public static final String MQTT_TOPIC_PROP = "trs.mqtt.broker";
    public static final String DEFAULT_SP_ID = "default";
    public static final String NS_SHACL = "http://www.w3.org/ns/shacl#";
    public static final String MIME_TURTLE = "text/turtle";

    public static final String MQTT_CLIENT_ID = "trs-consumer-whc";
    public static final String KB_UPDATE_PROP = "kb.update_uri";

    public static ServletContext getContext() {
        return context;
    }

    public static void setContext(final ServletContext context) {
        AdaptorHelper.context = context;
    }

    private static ServletContext context;


    public static String p(final String s) {
        final String parameterFQDN = parameterFQDN(s);
        log.debug("Retrieving the context init param '{}'", parameterFQDN);
        final String value = context.getInitParameter(parameterFQDN);
        log.trace("{}={}", parameterFQDN, value);
        return value;
    }

    /**
     * This returned value is not really used for the MQTT connection init.
     *
     * @see WarehouseControllerManager#getMqttClientId()
     */
    @Deprecated
    public static String getMqttClientId() {
        // FIXME Andrew@2019-01-29: what about the generated UUID?!
        return MQTT_CLIENT_ID;
    }


    static String parameterFQDN(final String s) {
        return PACKAGE_ROOT + "." + s;
    }

    public static Model loadJenaModelFromResource(final String resourceName, final Lang lang) {
        final InputStream resourceAsStream = WarehouseControllerManager.class.getClassLoader()
            .getResourceAsStream(resourceName);
        final Model problemModel = ModelFactory.createDefaultModel();
        RDFDataMgr.read(problemModel, resourceAsStream, lang);
        return problemModel;
    }

    public static void initLogger() {
        SLF4JBridgeHandler.removeHandlersForRootLogger();
        SLF4JBridgeHandler.install();
        java.util.logging.Logger.getLogger("").setLevel(Level.FINEST);
    }

    @NotNull
    public static ServiceProviderInfo[] defaultSPInfo() {
        final ServiceProviderInfo[] serviceProviderInfos;
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = DEFAULT_SP_ID;
        serviceProviderInfo.name = "Default Service Provider";
        serviceProviderInfos = new ServiceProviderInfo[]{serviceProviderInfo};
        return serviceProviderInfos;
    }

}
