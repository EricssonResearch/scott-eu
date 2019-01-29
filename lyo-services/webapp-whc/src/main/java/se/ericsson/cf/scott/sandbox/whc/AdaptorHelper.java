package se.ericsson.cf.scott.sandbox.whc;

import java.io.InputStream;
import java.io.StringWriter;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.logging.Level;
import javax.servlet.ServletContext;
import javax.xml.datatype.DatatypeConfigurationException;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.rdf.model.ModelFactory;
import org.apache.jena.rdf.model.ResIterator;
import org.apache.jena.rdf.model.Resource;
import org.apache.jena.riot.Lang;
import org.apache.jena.riot.RDFDataMgr;
import org.apache.jena.riot.RDFFormat;
import org.apache.jena.util.ResourceUtils;
import org.eclipse.lyo.oslc4j.core.exception.OslcCoreApplicationException;
import org.eclipse.lyo.oslc4j.core.model.IResource;
import org.eclipse.lyo.oslc4j.core.model.Link;
import org.eclipse.lyo.oslc4j.provider.jena.JenaModelHelper;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.slf4j.bridge.SLF4JBridgeHandler;

/**
 * TODO
 *
 * @version $version-stub$
 * @since   TODO
 */
public class AdaptorHelper {
    private final static Logger log = LoggerFactory.getLogger(AdaptorHelper.class);
    // Start of user code class_attributes
    private final static String PACKAGE_ROOT = WarehouseControllerManager.class.getPackage().getName();
    // FIXME Andrew@2018-07-30: extract into AdaptorConfig (non-static)
    public static final String MQTT_BROKER_PNAME = "trs.mqtt.broker";
    public static final String DEFAULT_SP_ID = "default";
    public static final String NS_SHACL = "http://www.w3.org/ns/shacl#";
    public static final String MIME_TURTLE = "text/turtle";

    // TODO Andrew@2019-01-29: clean up
    //    public static final String SPARQL_QUERY_URI = "https://aide.md.kth.se/fuseki/trs-everywhere/sparql";
//    public static final String SPARQL_UPDATE_URI = "https://aide.md.kth.se/fuseki/trs-everywhere/update";
//    public static final String PLAN_CF_URI = "http://aide.md.kth.se:3020/planner/planCreationFactory";
    // TODO Andrew@2019-01-29: comment out
    public static final String MQTT_CLIENT_ID = "trs-consumer-whc";
    static ServletContext context;

    // TODO Andrew@2018-02-07: submit to the JenaModelHelper
    // TODO Andrew@2018-07-30: replace with 2.4.0 calls
    public static <T> T[] fromJenaModelTyped(final Model model, Class<T> clazz) {
        try {
            final Object[] objects = JenaModelHelper.fromJenaModel(model, clazz);
            //noinspection unchecked
            final T[] clazzObjects = (T[]) objects;
            return clazzObjects;
        } catch (DatatypeConfigurationException | IllegalAccessException | InvocationTargetException | InstantiationException | OslcCoreApplicationException
                | NoSuchMethodException | URISyntaxException e) {
            throw new IllegalArgumentException(e);
        }
    }

    // TODO Andrew@2018-02-07: submit to the JenaModelHelper
    // TODO Andrew@2018-07-30: replace with 2.4.0 calls
    public static <T> T fromJenaModelSingle(final Model model, Class<T> clazz) {
        final T[] ts = fromJenaModelTyped(model, clazz);
        if (ts.length != 1) {
            throw new IllegalArgumentException(
                    "Model shall contain exactly 1 instance of the class");
        }
        return ts[0];
    }

    // TODO Andrew@2018-07-30: check if can replace with 2.4.0 calls
    public static void skolemize(final Model m) {
        final ResIterator resIterator = m.listSubjects();
        while(resIterator.hasNext()) {
            final Resource resource = resIterator.nextResource();
            if(resource != null && resource.isAnon()) {
                final String skolemURI = "urn:skolem:" + resource.getId()
                        .getBlankNodeId()
                        .getLabelString();
                final Resource skolemizedResource = ResourceUtils.renameResource(resource,
                                                                                 skolemURI);
            }
        }
    }

    public static String jenaModelToString(final Model responsePlan) {
        final StringWriter stringWriter = new StringWriter();
        RDFDataMgr.write(stringWriter, responsePlan, RDFFormat.TURTLE_PRETTY);
        return stringWriter.toString();
    }

    public static String p(final String s) {
        final String parameterFQDN = parameterFQDN(s);
        log.debug("Retrieving the context init param '{}'", parameterFQDN);
        final String value = context.getInitParameter(parameterFQDN);
        log.trace("{}={}", parameterFQDN, value);
        return value;
    }

    public static String getMqttClientId() {
        // FIXME Andrew@2019-01-29: what about the generated UUID?!
        return MQTT_CLIENT_ID;
    }

    // TODO Andrew@2018-02-23: move to JMH
    // TODO Andrew@2018-02-23: create a stateful JMH that would keep resources hashed by URI
    // TODO Andrew@2018-07-30: move to LyoHelper
    private static <R extends IResource> R nav(final Model m, final Link l,
            final Class<R> rClass) {
        final R[] rs = fromJenaModelTyped(m, rClass);
        for (R r : rs) {
            if(l.getValue().equals(r.getAbout())) {
                return r;
            }
        }
        throw new IllegalArgumentException("Link cannot be followed in this model");
    }

    // TODO Andrew@2018-07-30: move to LyoHelper
    @SafeVarargs
    public static IResource navTry(final Model m, final Link l,
            final Class<? extends IResource>... rClass) {
        for (Class<? extends IResource> aClass : rClass) {
            try {
                return nav(m, l, aClass);
            } catch (IllegalArgumentException e) {
                log.warn("Fix RDFS reasoning in JMH!!!");
            }
        }
        // give up
        throw new IllegalArgumentException("Link cannot be followed in this model");
    }

    static String parameterFQDN(final String s) {
        return PACKAGE_ROOT + "." + s;
    }

    public static Model loadJenaModelFromResource(final String resourceName, final Lang lang) {
        final InputStream resourceAsStream = WarehouseControllerManager.class.getClassLoader().getResourceAsStream(
                resourceName);
        final Model problemModel = ModelFactory.createDefaultModel();
        RDFDataMgr.read(problemModel, resourceAsStream, lang);
        return problemModel;
    }

    static void initLogger() {
        // TODO Andrew@2018-07-28: document this
        SLF4JBridgeHandler.removeHandlersForRootLogger();
        SLF4JBridgeHandler.install();
        java.util.logging.Logger.getLogger("").setLevel(Level.FINEST);
    }

    @NotNull
    static ServiceProviderInfo[] defaultSPInfo() {
        final ServiceProviderInfo[] serviceProviderInfos;
        final ServiceProviderInfo serviceProviderInfo = new ServiceProviderInfo();
        serviceProviderInfo.serviceProviderId = DEFAULT_SP_ID;
        serviceProviderInfo.name = "Default Service Provider";
        serviceProviderInfos = new ServiceProviderInfo[]{serviceProviderInfo};
        return serviceProviderInfos;
    }

    static String hexHashCodeFor(final IResource aResource) {
        return Integer.toHexString(aResource.hashCode());
    }
}
